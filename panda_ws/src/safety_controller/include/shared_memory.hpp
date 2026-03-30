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
};

struct OutputState {
	double safe_cmd_linear[3] = {0.0, 0.0, 0.0};
	double safe_cmd_angular[3] = {0.0, 0.0, 0.0};
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
