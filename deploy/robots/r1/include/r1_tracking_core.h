#pragma once

// SDK-independent tracking engine; shared by the hardware FSM and offline tests.
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <onnxruntime_cxx_api.h>
#include <array>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace r1_tracking {
using Joints = Eigen::Matrix<float,24,1>;
using Observation = Eigen::Matrix<float,135,1>;
struct State {
    Eigen::Vector3f position, velocity_body, angular_velocity_body;
    Eigen::Quaternionf orientation;
    Joints q, dq;
};
struct Reference {
    Joints q, dq;
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
};
inline void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
inline Eigen::Quaternionf quaternion(const YAML::Node& node) {
    auto v=node.as<std::vector<float>>();
    require(v.size()==4,"Expected wxyz quaternion");
    Eigen::Quaternionf q(v[0],v[1],v[2],v[3]);
    require(q.coeffs().allFinite() && std::abs(q.norm()-1.f)<.01f,"Invalid quaternion");
    return q.normalized();
}
template<int N> Eigen::Matrix<float,N,1> vector(const YAML::Node& node) {
    auto v=node.as<std::vector<float>>();
    require(v.size()==N,"Unexpected vector length");
    Eigen::Matrix<float,N,1> out=Eigen::Map<Eigen::Matrix<float,N,1>>(v.data());
    require(out.allFinite(),"Nonfinite input");
    return out;
}
inline Eigen::Quaternionf yaw(const Eigen::Quaternionf& q) {
    auto r=q.toRotationMatrix();
    return Eigen::Quaternionf(Eigen::AngleAxisf(std::atan2(r(1,0),r(0,0)),Eigen::Vector3f::UnitZ()));
}
inline std::pair<Eigen::Vector3f,Eigen::Quaternionf> torso(const State& s) {
    auto roll=Eigen::Quaternionf(Eigen::AngleAxisf(s.q[12],Eigen::Vector3f::UnitX()));
    auto turn=Eigen::Quaternionf(Eigen::AngleAxisf(s.q[13],Eigen::Vector3f::UnitZ()));
    return {s.position+s.orientation*(roll*Eigen::Vector3f(.0325f,0,.049f)),s.orientation*roll*turn};
}
class Engine {
public:
    YAML::Node config;
    Joints offset, scale, kp, kd, effort, lower, upper;
    std::vector<int> motor_ids;
    std::vector<Reference> reference;
    float dt;
    Eigen::Quaternionf alignment=Eigen::Quaternionf::Identity();
    Eigen::Vector3f translation=Eigen::Vector3f::Zero();

    explicit Engine(const std::filesystem::path& yaml):config(YAML::LoadFile(yaml.string())) {
        require(config["controller"].as<std::string>()=="r1_motion_tracking_v1","Wrong deployment controller");
        motor_ids=config["joint_ids_map"].as<std::vector<int>>();
        require(motor_ids==std::vector<int>({0,1,2,3,4,5,6,7,8,9,10,11,12,13,15,16,17,18,19,22,23,24,25,26}),"R1 motor mapping mismatch");
        dt=config["step_dt"].as<float>(); require(std::abs(dt-.02f)<1e-7f,"Policy requires 50 Hz");
        offset=vector<24>(config["default_joint_pos"]); scale=vector<24>(config["actions"]["scale"]);
        kp=vector<24>(config["stiffness"]); kd=vector<24>(config["damping"]);
        effort=vector<24>(config["effort_limits"]);
        lower=vector<24>(config["joint_limits_lower"]); upper=vector<24>(config["joint_limits_upper"]);
        require((kp.array()>0).all() && (kd.array()>=0).all() && (scale.array()>0).all() && (effort.array()>0).all(),"Invalid control parameters");
        require((lower.array()<upper.array()).all(),"Invalid joint ranges");
        require(config["head_motor_ids"].as<std::vector<int>>()==std::vector<int>({29,30}),"R1 head mapping mismatch");
        require(config["actions"]["clip"].IsNull(),"This policy uses unclipped actions");
        require((vector<24>(config["actions"]["offset"])-offset).cwiseAbs().maxCoeff()<1e-7f,"Action offset mismatch");
        require(config["state_estimation"]["world_z_ground_m"].as<float>()==0.f,"Estimator must use floor z=0");
        auto base=yaml.parent_path();
        std::ifstream file(base/config["motion_file"].as<std::string>(),std::ios::binary);
        require(bool(file),"Cannot open reference.bin");
        char magic[8]; uint32_t count=0,width=0;
        file.read(magic,8);file.read(reinterpret_cast<char*>(&count),4);file.read(reinterpret_cast<char*>(&width),4);
        require(std::string(magic,8)=="R1REF001" && count>1 && count<1000000 && width==55,"Invalid reference header");
        require(config["reference_frames"].as<uint32_t>()==count,"Reference length mismatch");
        for(uint32_t i=0;i<count;++i) {
            std::array<float,55> row{};file.read(reinterpret_cast<char*>(row.data()),sizeof(row));
            require(bool(file),"Truncated reference");
            Reference f;f.q=Eigen::Map<Joints>(row.data());f.dq=Eigen::Map<Joints>(row.data()+24);
            f.position=Eigen::Map<Eigen::Vector3f>(row.data()+48);
            f.orientation=Eigen::Quaternionf(row[51],row[52],row[53],row[54]);
            require(f.q.allFinite() && f.dq.allFinite() && f.position.allFinite() && f.orientation.coeffs().allFinite() && std::abs(f.orientation.norm()-1)<.001,"Invalid reference values");
            f.orientation.normalize();reference.push_back(f);
        }
        require(file.peek()==std::ifstream::traits_type::eof(),"Trailing reference bytes");
        options.SetIntraOpNumThreads(1);options.SetInterOpNumThreads(1);
        session=std::make_unique<Ort::Session>(environment,(base/config["policy_file"].as<std::string>()).c_str(),options);
        require(session->GetInputCount()==1 && session->GetOutputCount()==1,"Expected plain policy.onnx");
        auto input_type=session->GetInputTypeInfo(0), output_type=session->GetOutputTypeInfo(0);
        auto in=input_type.GetTensorTypeAndShapeInfo();
        auto out=output_type.GetTensorTypeAndShapeInfo();
        require(in.GetShape()==std::vector<int64_t>({1,135}) && out.GetShape()==std::vector<int64_t>({1,24}),"Policy interface mismatch");
        Ort::AllocatorWithDefaultOptions allocator;
        require(std::string(session->GetInputNameAllocated(0,allocator).get())=="obs" && std::string(session->GetOutputNameAllocated(0,allocator).get())=="actions","Policy tensor names mismatch");
    }
    void align(const State& s) {
        auto a=torso(s);alignment=yaw(a.second*reference[0].orientation.conjugate());
        translation=a.first-alignment*reference[0].position;translation.z()=0;
    }
    Observation observe(size_t frame,const State& s,const Joints& last) const {
        require(frame<reference.size(),"Reference frame out of range");
        require(s.position.allFinite() && s.velocity_body.allFinite() && s.angular_velocity_body.allFinite() && s.q.allFinite() && s.dq.allFinite() && last.allFinite(),"Nonfinite robot state");
        require(s.orientation.coeffs().allFinite() && std::abs(s.orientation.norm()-1)<.01,"Invalid robot orientation");
        const auto& f=reference[frame];auto a=torso(s);
        auto p=a.second.conjugate()*(alignment*f.position+translation-a.first);
        Eigen::Matrix3f rot=(a.second.conjugate()*alignment*f.orientation).toRotationMatrix();
        Observation obs;obs.segment<24>(0)=f.q;obs.segment<24>(24)=f.dq;obs.segment<3>(48)=p;
        for(int row=0;row<3;++row)for(int col=0;col<2;++col)obs[51+row*2+col]=rot(row,col);
        obs.segment<3>(57)=s.velocity_body;obs.segment<3>(60)=s.angular_velocity_body;
        obs.segment<24>(63)=s.q-offset;obs.segment<24>(87)=s.dq;obs.segment<24>(111)=last;
        require(obs.allFinite(),"Nonfinite observation");return obs;
    }
    Joints infer(Observation obs) {
        const std::array<int64_t,2> shape{1,135};const char* inputs[]={"obs"};const char* outputs[]={"actions"};
        auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
        auto tensor=Ort::Value::CreateTensor<float>(memory,obs.data(),135,shape.data(),2);
        auto values=session->Run(Ort::RunOptions{nullptr},inputs,&tensor,1,outputs,1);
        Joints result=Eigen::Map<const Joints>(values[0].GetTensorData<float>());
        require(result.allFinite(),"Nonfinite policy action");return result;
    }
    Joints targets(const Joints& action) const {return offset+scale.cwiseProduct(action);}
private:
    Ort::Env environment{ORT_LOGGING_LEVEL_WARNING,"r1_tracking"};
    Ort::SessionOptions options;
    std::unique_ptr<Ort::Session> session;
};
inline State parse_state(const YAML::Node& n) {
    State s;s.position=vector<3>(n["position_w"]);s.orientation=quaternion(n["quaternion_wxyz"]);
    s.velocity_body=vector<3>(n["linear_velocity_b"]);s.angular_velocity_body=vector<3>(n["angular_velocity_b"]);
    s.q=vector<24>(n["joint_pos"]);s.dq=vector<24>(n["joint_vel"]);return s;
}
} // namespace r1_tracking
