#include "State_Tracking.h"
#include <algorithm>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <limits>
#include <unitree/dds_wrapper/robots/r1/defines.h>

static_assert(int(unitree::robot::r1::JointIndex::WaistRoll)==12);
static_assert(int(unitree::robot::r1::JointIndex::RightWristRoll)==26);
static_assert(int(unitree::robot::r1::JointIndex::HeadPitch)==29);
static_assert(int(unitree::robot::r1::JointIndex::HeadYaw)==30);

namespace {
int64_t now_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
}
State_Tracking::State_Tracking(int id,std::string name):FSMState(id,name) {
    auto directory=param::parser_policy_dir(param::config["FSM"][name]["policy_dir"].as<std::string>());
    engine_=std::make_unique<r1_tracking::Engine>(directory/"params/deploy.yaml");
    socket_=::socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0);
    if(socket_<0)throw std::runtime_error("Cannot create odometry socket");
    sockaddr_in address{};address.sin_family=AF_INET;
    auto estimation=engine_->config["state_estimation"];
    feed_=std::make_unique<r1_tracking::PoseFeed>(estimation["max_age_s"].as<double>());
    r1_tracking::require(estimation["bind_address"].as<std::string>()=="127.0.0.1","Odometry bridge must run on the controller host");
    address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);address.sin_port=htons(estimation["port"].as<int>());
    if(::bind(socket_,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0){::close(socket_);socket_=-1;throw std::runtime_error("Cannot bind odometry socket");}
    instance_=this;
}
State_Tracking::~State_Tracking(){if(instance_==this)instance_=nullptr;if(socket_>=0)::close(socket_);}
bool State_Tracking::auto_entry_ready(std::string& reason) {
    if(!instance_){reason="tracking state is not initialized";return false;}
    try {
        instance_->receive_estimate();instance_->read_robot();instance_->engine_->align(instance_->state_);
        const auto measured=r1_tracking::torso(instance_->state_);
        const float joint_error=(instance_->state_.q-instance_->engine_->reference[0].q).cwiseAbs().maxCoeff();
        if(joint_error>instance_->engine_->config["entry"]["max_joint_error_rad"].as<float>()) {
            reason="first-reference joint pose still outside entry threshold";return false;
        }
        const float height_error=std::abs(measured.first.z()-instance_->engine_->reference[0].position.z());
        if(height_error>instance_->engine_->config["entry"]["max_height_error_m"].as<float>()) {
            reason="pelvis height still outside entry threshold";return false;
        }
        reason.clear();return true;
    } catch(const std::exception& e) {reason=e.what();return false;}
}
void State_Tracking::receive_estimate() {
    char buffer[4096];
    // Bound processing time even if a sender floods this socket.
    for(int packet=0;packet<32;++packet) {
        auto length=::recv(socket_,buffer,sizeof(buffer),0);if(length<0)break;
        try {
            auto n=YAML::Load(std::string(buffer,length));
            feed_->accept(n,now_ns());
        }catch(const std::exception&){/* Invalid packets never refresh freshness. */}
    }
    feed_->require_fresh(now_ns());
    state_.position=feed_->position;state_.velocity_body=feed_->velocity;state_.orientation=feed_->orientation;
}
void State_Tracking::read_robot() {
    std::lock_guard<std::mutex> lock(lowstate->mutex_);
    for(int i=0;i<24;++i){const auto& m=lowstate->msg_.motor_state()[engine_->motor_ids[i]];state_.q[i]=m.q();state_.dq[i]=m.dq();}
    for(int i=0;i<3;++i)state_.angular_velocity_body[i]=lowstate->msg_.imu_state().gyroscope()[i];
    r1_tracking::require(state_.q.allFinite() && state_.dq.allFinite() && state_.angular_velocity_body.allFinite(),"Invalid low-level state");
    for(int i=0;i<24;++i) {
        if(state_.q[i]<engine_->lower[i]-.1f || state_.q[i]>engine_->upper[i]+.1f) {
            const auto now=Clock::now();
            if(now>=next_range_warning_) {
                spdlog::warn("Measured joint {} (motor {}) at {:.3f} rad is outside configured range [{:.3f}, {:.3f}]; continuing tracking",i,engine_->motor_ids[i],state_.q[i],engine_->lower[i],engine_->upper[i]);
                next_range_warning_=now+std::chrono::seconds(1);
            }
            break;
        }
    }
}
void State_Tracking::latch_hold_pose() {
    if(hold_latched_)return;
    std::lock_guard<std::mutex> lock(lowstate->mutex_);
    const auto& measured=lowstate->msg_.motor_state();
    const auto& commands=lowcmd->msg_.motor_cmd();
    hold_q_.resize(commands.size());hold_kp_.resize(commands.size());hold_kd_.resize(commands.size());
    for(size_t i=0;i<commands.size();++i){
        // Preserve the last motor command at a fault. Jumping the target to the
        // measured pose while the robot is moving produces a torque step.
        const float q=i<measured.size()?measured[i].q():std::numeric_limits<float>::quiet_NaN();
        hold_q_[i]=std::isfinite(commands[i].q())?commands[i].q():(std::isfinite(q)?q:0.f);
        hold_kp_[i]=std::isfinite(commands[i].kp())?commands[i].kp():0.f;
        hold_kd_[i]=std::isfinite(commands[i].kd())?commands[i].kd():0.f;
    }
    hold_latched_=true;
    publish_hold_pose();
}
void State_Tracking::publish_hold_pose() {
    if(!hold_latched_)return;
    auto& commands=lowcmd->msg_.motor_cmd();
    for(size_t i=0;i<commands.size()&&i<hold_q_.size();++i){auto& m=commands[i];m.q()=hold_q_[i];m.kp()=hold_kp_[i];m.kd()=hold_kd_[i];m.dq()=0;m.tau()=0;}
}
void State_Tracking::publish_target() {
    for(int i=0;i<24;++i){auto& m=lowcmd->msg_.motor_cmd()[engine_->motor_ids[i]];m.q()=std::clamp(target_[i],engine_->lower[i],engine_->upper[i]);m.kp()=engine_->kp[i];m.kd()=engine_->kd[i];m.dq()=0;m.tau()=0;}
}
void State_Tracking::enter() {
    fault_=false;finished_=false;hold_latched_=false;frame_=0;previous_.setZero();next_range_warning_=Clock::time_point{};
    try {
        receive_estimate();read_robot();engine_->align(state_);
        auto measured=r1_tracking::torso(state_);
        r1_tracking::require((state_.q-engine_->reference[0].q).cwiseAbs().maxCoeff()<=engine_->config["entry"]["max_joint_error_rad"].as<float>(),"First-reference pose not reached");
        r1_tracking::require(std::abs(measured.first.z()-engine_->reference[0].position.z())<=engine_->config["entry"]["max_height_error_m"].as<float>(),"Ground/height calibration mismatch");
        // Hold the two head motors at entry; no reference head actions are trained.
        {std::lock_guard<std::mutex> lock(lowstate->mutex_);
         for(int id:{29,30}){auto& m=lowcmd->msg_.motor_cmd()[id];float q=lowstate->msg_.motor_state()[id].q();r1_tracking::require(std::isfinite(q),"Invalid head state");m.q()=q;m.kp()=5;m.kd()=1;m.dq()=0;m.tau()=0;}}
        target_=state_.q;next_=Clock::now();
    }catch(const std::exception& e){fault_=true;latch_hold_pose();spdlog::error("Tracking entry rejected; holding last motor targets: {}",e.what());}
}
void State_Tracking::run() {
    if(fault_ || finished_){publish_hold_pose();return;}
    try {
        receive_estimate();
        if(Clock::now()>=next_) {
            r1_tracking::require(std::chrono::duration<double>(Clock::now()-next_).count()<=engine_->config["stop"]["max_policy_lateness_s"].as<double>(),"Tracking deadline missed");
            if(frame_>=engine_->reference.size()-1){finished_=true;latch_hold_pose();spdlog::info("Tracking complete; holding final motor targets");return;}
            read_robot();auto obs=engine_->observe(frame_,state_,previous_);
            auto measured=r1_tracking::torso(state_);auto ref=engine_->reference[frame_];
            r1_tracking::require(std::abs(measured.first.z()-ref.position.z())<=engine_->config["stop"]["max_anchor_height_error_m"].as<float>(),"Anchor height gate");
            auto error=measured.second.conjugate()*engine_->alignment*ref.orientation;
            r1_tracking::require(2*std::acos(std::clamp(std::abs(error.w()),0.f,1.f))<=engine_->config["stop"]["max_anchor_orientation_error_rad"].as<float>(),"Anchor orientation gate");
            previous_=engine_->infer(obs);target_=engine_->targets(previous_);++frame_;
            next_+=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(engine_->dt));
        }
        publish_target();
    }catch(const std::exception& e){fault_=true;latch_hold_pose();spdlog::error("Tracking fault; holding last motor targets: {}",e.what());}
}
void State_Tracking::exit(){latch_hold_pose();}
