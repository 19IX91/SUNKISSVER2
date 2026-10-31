#pragma once
#include "FSM/FSMState.h"
#include "r1_tracking_core.h"
#include "r1_tracking_estimate.h"
#include <chrono>

class State_Tracking : public FSMState {
public:
    State_Tracking(int id,std::string name);
    ~State_Tracking();
    static bool auto_entry_ready(std::string& reason);
    void enter() override;
    void run() override;
    void exit() override;
private:
    using Clock=std::chrono::steady_clock;
    std::unique_ptr<r1_tracking::Engine> engine_;
    int socket_=-1;
    bool fault_=false,finished_=false;
    std::unique_ptr<r1_tracking::PoseFeed> feed_;
    r1_tracking::State state_;
    r1_tracking::Joints previous_=r1_tracking::Joints::Zero(),target_=r1_tracking::Joints::Zero();
    Clock::time_point next_;
    size_t frame_=0;
    Clock::time_point next_range_warning_{};
    inline static State_Tracking* instance_=nullptr;
    void receive_estimate();
    void read_robot();
    void latch_hold_pose();
    void publish_hold_pose();
    void publish_target();
    bool hold_latched_=false;
    std::vector<float> hold_q_,hold_kp_,hold_kd_;
};
REGISTER_FSM(State_Tracking)
