#pragma once

#include <atomic>
#include <cstdint>

namespace safety_controller {

struct InputState {
	double lidar_range = 0.0;
	double lidar_dir_base[3] = {0.0, 0.0, 1.0};
	double cmd_linear[3] = {0.0, 0.0, 0.0};
	double cmd_angular[3] = {0.0, 0.0, 0.0};
	double joint_pos[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	bool has_joint_state = false;
	bool has_lidar_dir = false;
	// steady_clock nanoseconds at the moment the bridge last refreshed this
	// state. Used by the RT watchdog to detect a stalled input stream.
	// 0 means "never written" and is treated as stale.
	uint64_t stamp_ns = 0;
};

struct OutputState {
	double safe_cmd_linear[3] = {0.0, 0.0, 0.0};
	double safe_cmd_angular[3] = {0.0, 0.0, 0.0};
};

// Tunable safety-law parameters. Defaults are sane starting points; the bridge
// overrides them from ROS parameters at startup.
struct SafetyParams {
	double stop_distance = 0.005;    // [m] latch to a near-stop below this range
	double slow_distance = 0.05;     // [m] begin scaling motion down below this range
	double release_margin = 0.005;   // [m] extra clearance required to release the stop latch
	double min_scale = 0.02;         // minimum velocity scale (never fully zero while active)
	double filter_alpha = 0.2;       // low-pass factor on the range signal (per tick)
	double lateral_weight = 0.3;     // fraction of slowdown applied to lateral/retreating motion
	double max_linear_vel = 0.25;    // [m/s] absolute clamp on each linear component
	double max_angular_vel = 1.0;    // [rad/s] absolute clamp on each angular component
	double input_timeout_s = 0.2;    // [s] zero the output if inputs are older than this
};

// Persistent state for the safety law (range filter + stop latch). Kept out of
// the compute function so it is reentrant and unit-testable.
struct SafetyState {
	bool range_initialized = false;
	double filtered_range = 0.0;
	bool stopped = false;
};

// Single-slot lock-free buffer for latest-value semantics.
template <typename T>
class LatestValueBuffer {
public:
	LatestValueBuffer() = default;

	void write(const T& value) noexcept {
		seq_.fetch_add(1, std::memory_order_relaxed);
		data_ = value;
		seq_.fetch_add(1, std::memory_order_release);
	}

	T read() const noexcept {
		T snapshot;
		for (;;) {
			uint64_t seq1 = seq_.load(std::memory_order_acquire);
			snapshot = data_;
			uint64_t seq2 = seq_.load(std::memory_order_acquire);
			if (seq1 == seq2 && (seq1 % 2u) == 0u) {
				break;
			}
		}
		return snapshot;
	}

private:
	mutable std::atomic<uint64_t> seq_{0};
	T data_{};
};

struct SharedMemory {
	LatestValueBuffer<InputState> inputs;
	LatestValueBuffer<OutputState> outputs;
};

}  // namespace safety_controller
