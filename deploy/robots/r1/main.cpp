#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Tracking.h"
#include "State_FixStandSunkiss.h"
#include "State_ActiveHold.h"
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <cstdlib>

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     R1 Controller \n";

    if (vm.count("check-config")) {
        auto config_type = param::config["FSM"]["_"]["FixStand"]["type"].as<std::string>();
        if (config_type != "FixStandSunkiss")
            throw std::runtime_error("Sunkiss config must select State_FixStandSunkiss");
        auto directory = param::parser_policy_dir(param::config["FSM"]["Tracking"]["policy_dir"].as<std::string>());
        r1_tracking::Engine engine(directory / "params/deploy.yaml");
        std::cout << "Sunkiss config valid; reference frames=" << engine.reference.size()
                  << ", policy dt=" << engine.dt << " s; DDS not initialized, no commands sent.\n";
        return 0;
    }

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    // R1 low-level control requires releasing the active high-level motion owner.
    // CheckMode is read-only; ReleaseMode is only sent when an owner is reported.
    auto motion_switcher = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
    motion_switcher->SetTimeout(3.0f);
    motion_switcher->Init();
    std::string motion_form, motion_name;
    for (int attempt = 0; attempt < 8; ++attempt) {
        if (motion_switcher->CheckMode(motion_form, motion_name) != 0)
            throw std::runtime_error("MotionSwitcher CheckMode failed; refusing to publish lowcmd");
        if (motion_name.empty()) break;
        spdlog::warn("Releasing active motion owner '{}' before low-level control", motion_name);
        if (motion_switcher->ReleaseMode() != 0)
            throw std::runtime_error("MotionSwitcher ReleaseMode failed; refusing to publish lowcmd");
        usleep(250000);
    }
    if (motion_switcher->CheckMode(motion_form, motion_name) != 0 || !motion_name.empty())
        throw std::runtime_error("Motion owner did not release; refusing to publish lowcmd");

    init_fsm_state();

    FSMState::lowcmd->msg_.mode_machine() = 1;
    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }

    // Match the previously used R1 walk handoff: seed the first active
    // low-level target from the measured pose before starting the FSM. This
    // avoids entering Passive (kp=0) after MotionSwitcher releases the onboard
    // controller, which would drop active support before the pose blend.
    {
        std::lock_guard<std::mutex> lock(FSMState::lowstate->mutex_);
        const auto& measured = FSMState::lowstate->msg_.motor_state();
        auto& command = FSMState::lowcmd->msg_.motor_cmd();
        if (measured.size() < command.size())
            throw std::runtime_error("LowState motor vector is shorter than LowCmd; refusing to start");
        for (size_t i = 0; i < command.size(); ++i) {
            const float q = measured[i].q();
            if (!std::isfinite(q))
                throw std::runtime_error("Non-finite measured motor pose; refusing to start");
            command[i].q() = q;
        }
    }
    
    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    const char* auto_start = std::getenv("R1_SUNKISS_AUTO_START");
    const bool automatic = auto_start && (std::string(auto_start) == "1" || std::string(auto_start) == "true");
    std::cout << "Starting directly in FixStand, seeded from the measured joint pose.\n";
    if (automatic)
        std::cout << "Sunkiss auto-start enabled: after the pose blend and countdown, tracking starts without RT+A. Press [LT + B] to cancel.\n";
    else
        std::cout << "Press [RT + A] to start the Sunkiss tracking sequence.\n";

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}

