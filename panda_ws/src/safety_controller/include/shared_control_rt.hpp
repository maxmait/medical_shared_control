#pragma once

#include "shared_memory.hpp"

#include <atomic>
#include <chrono>
#include <thread>

namespace safety_controller {

class SharedControlRt {
public:
    SharedControlRt(SharedMemory& shared, double rate_hz);
    ~SharedControlRt();

    void start();
    void stop();

private:
    void loop();

    SharedMemory& shared_;
    std::chrono::steady_clock::duration period_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace safety_controller
