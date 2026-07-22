#pragma once

#include "shared_memory.hpp"

#include <atomic>
#include <chrono>
#include <thread>

namespace safety_controller {

// Pure safety law: maps a user command to a safe command given the tunable
// parameters and the persistent filter/latch state. `input_age_s` is how old the
// input is (seconds); when it exceeds `params.input_timeout_s` the output is
// zeroed (dead-man watchdog). Kept free of ROS and of any I/O so it can be unit
// tested directly. `state` is read and updated in place.
OutputState compute_safe_command(const InputState& in,
                                 const SafetyParams& params,
                                 SafetyState& state,
                                 double input_age_s);

class SharedControlRt {
public:
    SharedControlRt(SharedMemory& shared, double rate_hz, const SafetyParams& params = SafetyParams{});
    ~SharedControlRt();

    void start();
    void stop();

    // Replace the active parameters (thread-safe: guarded by an atomic seq copy).
    void set_params(const SafetyParams& params);

private:
    void loop();

    SharedMemory& shared_;
    std::chrono::steady_clock::duration period_;
    LatestValueBuffer<SafetyParams> params_;
    SafetyState state_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace safety_controller
