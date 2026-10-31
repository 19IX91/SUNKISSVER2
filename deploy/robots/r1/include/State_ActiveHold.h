#pragma once

#include "FSM/FSMState.h"
#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <vector>

// Holds the measured pose with active gains after a Sunkiss fault or timeout.
// It preserves joint posture; it is not a balance recovery controller.
class State_ActiveHold : public FSMState {
public:
    State_ActiveHold(int id, std::string name = "ActiveHold") : FSMState(id, name) {
        kp_ = param::config["FSM"]["FixStand"]["kp"].as<std::vector<float>>();
        kd_ = param::config["FSM"]["FixStand"]["kd"].as<std::vector<float>>();
        if (kp_.size() != kd_.size() || kp_.empty())
            throw std::runtime_error("ActiveHold requires matching FixStand gain vectors");
        for (auto& gain : kp_) gain = std::max(gain, 5.f);
        for (auto& gain : kd_) gain = std::max(gain, 1.f);
    }
    void enter() override {
        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        const auto& measured = lowstate->msg_.motor_state();
        auto& commands = lowcmd->msg_.motor_cmd();
        if (measured.size() < commands.size() || kp_.size() < commands.size())
            throw std::runtime_error("ActiveHold motor state/gain vector is incomplete");
        q_.resize(commands.size());
        for (size_t i = 0; i < commands.size(); ++i) {
            const float q = measured[i].q();
            if (!std::isfinite(q)) throw std::runtime_error("ActiveHold received non-finite joint position");
            q_[i] = q;
        }
        publish();
    }
    void run() override { publish(); }
private:
    std::vector<float> kp_, kd_, q_;
    void publish() {
        if (q_.empty()) return;
        auto& commands = lowcmd->msg_.motor_cmd();
        for (size_t i = 0; i < commands.size() && i < q_.size(); ++i) {
            auto& m = commands[i]; m.q() = q_[i]; m.kp() = kp_[i]; m.kd() = kd_[i]; m.dq() = 0.f; m.tau() = 0.f;
        }
    }
};
REGISTER_FSM(State_ActiveHold)
