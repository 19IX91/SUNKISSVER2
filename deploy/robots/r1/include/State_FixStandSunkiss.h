#pragma once

#include "FSM/State_FixStand.h"
#include "State_Tracking.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

// Sunkiss-only FixStand variant. The normal stock walking FixStand state is
// untouched. Set R1_SUNKISS_AUTO_START=1 to transition to Tracking after the
// configured FixStand interpolation and an optional countdown.
class State_FixStandSunkiss : public State_FixStand {
public:
    State_FixStandSunkiss(int id, std::string name = "FixStand")
        : State_FixStand(id, name) {
        auto times = param::config["FSM"][name]["ts"].as<std::vector<float>>();
        if (times.empty() || times.back() < 0.f)
            throw std::runtime_error("Sunkiss FixStand requires a valid final interpolation time");
        stand_seconds_ = times.back();
        auto flag = std::getenv("R1_SUNKISS_AUTO_START");
        auto_start_ = flag && (std::string(flag) == "1" || std::string(flag) == "true");
        countdown_seconds_ = read_countdown();
        if (auto_start_) {
            const int tracking = FSMStringMap.right.at("Tracking");
            registered_checks.erase(
                std::remove_if(registered_checks.begin(), registered_checks.end(),
                               [tracking](const auto& check) { return check.second == tracking; }),
                registered_checks.end());
            registered_checks.emplace_back([this]() { return countdown_complete(); }, tracking);
        }
    }

    void enter() override {
        State_FixStand::enter();
        entered_ = Clock::now();
        last_countdown_ = -1; countdown_started_=false; last_wait_log_=Clock::time_point{};
        if (auto_start_)
            spdlog::info("Sunkiss auto-start armed: FixStand blend {:.1f}s, then {}s countdown; LT+B cancels",
                         stand_seconds_, countdown_seconds_);
    }

    void run() override {
        State_FixStand::run();
        if (!auto_start_) return;
        const double elapsed = std::chrono::duration<double>(Clock::now() - entered_).count();
        if(elapsed<stand_seconds_)return;
        std::string reason;
        if(!State_Tracking::auto_entry_ready(reason)) {
            countdown_started_=false;
            const auto now=Clock::now();
            if(last_wait_log_==Clock::time_point{} || std::chrono::duration<double>(now-last_wait_log_).count()>=1.) {
                spdlog::info("Sunkiss auto-start waiting for tracking entry: {}",reason);
                last_wait_log_=now;
            }
            return;
        }
        if(!countdown_started_) {
            countdown_started_=true;countdown_at_=Clock::now();last_countdown_=-1;
            spdlog::info("Sunkiss entry checks pass; countdown started");
        }
        const double countdown_elapsed=std::chrono::duration<double>(Clock::now()-countdown_at_).count();
        const int remaining = static_cast<int>(std::ceil(countdown_seconds_ - countdown_elapsed));
        if (remaining >= 0 && remaining <= countdown_seconds_ && remaining != last_countdown_) {
            last_countdown_ = remaining;
            if (remaining > 0) spdlog::info("Sunkiss starts in {}s; LT+B cancels", remaining);
            else spdlog::info("Sunkiss auto-start: checking tracking entry conditions");
        }
    }

private:
    using Clock = std::chrono::steady_clock;
    bool auto_start_ = false;
    double stand_seconds_ = 0.;
    int countdown_seconds_ = 1;
    int last_countdown_ = -1;
    Clock::time_point entered_{};
    Clock::time_point countdown_at_{};
    Clock::time_point last_wait_log_{};
    bool countdown_started_=false;

    static int read_countdown() {
        const char* value = std::getenv("R1_SUNKISS_AUTO_START_DELAY");
        if (!value) return 1;
        char* end = nullptr;
        long seconds = std::strtol(value, &end, 10);
        if (!end || *end != '\0' || seconds < 0 || seconds > 30)
            throw std::runtime_error("R1_SUNKISS_AUTO_START_DELAY must be an integer from 0 to 30");
        return static_cast<int>(seconds);
    }

    bool countdown_complete() const {
        return auto_start_ && countdown_started_ &&
            std::chrono::duration<double>(Clock::now() - countdown_at_).count() >= countdown_seconds_;
    }
};

REGISTER_FSM(State_FixStandSunkiss)
