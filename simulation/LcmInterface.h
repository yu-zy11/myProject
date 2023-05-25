#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <thread>

#include "MujocoCommand.hpp"
#include "MujocoState.hpp"

class LcmInterface {
public:
    ~LcmInterface() { _subThread.join(); };
    void initLCM() {
        if (!_lc.good()) {
            std::cout << "lcm is not good";
        }
        _lc.subscribe("mujoco_state", &LcmInterface::subscribe_callback, this);
        _subThread = std::thread(&LcmInterface::lcmHandleThread, this);
    };
    void lcmPublishCommand() { _lc.publish("mujoco_cmd", &_mujoco_cmd); };
    void subscribe_callback(const lcm::ReceiveBuffer *rbuf, const std::string &channel, const MujocoState *msg) {
        memcpy(&_mujoco_state, msg, sizeof(MujocoState));
    };
    void lcmHandleThread() {
        while (true) {
            _lc.handle();
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    };
    lcm::LCM _lc;
    std::thread _subThread;
    MujocoCommand _mujoco_cmd;
    MujocoState _mujoco_state;
};