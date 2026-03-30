#include "shared_control_rt.hpp"

#include <cmath>
#include <cstdint>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

namespace safety_controller {

namespace {

void best_effort_realtime_setup(int priority) {
	mlockall(MCL_CURRENT | MCL_FUTURE);

	sched_param params;
	params.sched_priority = priority;
	pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);
}

OutputState compute_safe_command(const InputState& in) {
	OutputState out;

	// Placeholder safety: linearly scale down velocity as lidar approaches 0.05 m.
	const double stop_distance = 0.0033131339587271214;
	const double slow_distance = 0.05;
	const double release_margin = 0.005;
	const double min_scale = 0.02;
	const double filter_alpha = 0.2;
	const double lateral_weight = 0.3;

	static bool range_initialized = false;
	static double filtered_range = 0.0;
	static bool stopped = false;

	if (!range_initialized) {
		filtered_range = in.lidar_range;
		range_initialized = true;
	} else {
		filtered_range = (filter_alpha * in.lidar_range) + ((1.0 - filter_alpha) * filtered_range);
	}
	double scale = 1.0;

	if (stopped) {
		if (filtered_range >= stop_distance + release_margin) {
			stopped = false;
		} else {
			scale = min_scale;
		}
	}

	if (!stopped && filtered_range <= stop_distance) {
		stopped = true;
		scale = min_scale;
	} else if (!stopped && filtered_range < slow_distance) {
		scale = (filtered_range - stop_distance) / (slow_distance - stop_distance);
	}

	if (scale < min_scale) {
		scale = min_scale;
	}

	const double base_scale = scale;
	const double mild_scale = 1.0 - (lateral_weight * (1.0 - base_scale));

	if (in.has_lidar_dir) {
		double dir_x = in.lidar_dir_base[0];
		double dir_y = in.lidar_dir_base[1];
		double dir_z = in.lidar_dir_base[2];
		const double dir_norm = std::sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
		if (dir_norm > 1e-9) {
			dir_x /= dir_norm;
			dir_y /= dir_norm;
			dir_z /= dir_norm;
		}

		const double vx = in.cmd_linear[0];
		const double vy = in.cmd_linear[1];
		const double vz = in.cmd_linear[2];
		const double v_parallel = (vx * dir_x) + (vy * dir_y) + (vz * dir_z);

		const double v_perp_x = vx - (v_parallel * dir_x);
		const double v_perp_y = vy - (v_parallel * dir_y);
		const double v_perp_z = vz - (v_parallel * dir_z);

		const double v_perp_scaled_x = v_perp_x * mild_scale;
		const double v_perp_scaled_y = v_perp_y * mild_scale;
		const double v_perp_scaled_z = v_perp_z * mild_scale;

		const double v_parallel_scaled = (v_parallel > 0.0)
			? (v_parallel * base_scale)
			: (v_parallel * mild_scale);

		out.safe_cmd_linear[0] = v_perp_scaled_x + (v_parallel_scaled * dir_x);
		out.safe_cmd_linear[1] = v_perp_scaled_y + (v_parallel_scaled * dir_y);
		out.safe_cmd_linear[2] = v_perp_scaled_z + (v_parallel_scaled * dir_z);
	} else {
		for (int i = 0; i < 3; ++i) {
			out.safe_cmd_linear[i] = in.cmd_linear[i] * base_scale;
		}
	}

	for (int i = 0; i < 3; ++i) {
		out.safe_cmd_angular[i] = in.cmd_angular[i] * scale;
	}

	return out;
}

}  // namespace

SharedControlRt::SharedControlRt(SharedMemory& shared, double rate_hz)
	: shared_(shared),
	  period_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
		  std::chrono::duration<double>(1.0 / rate_hz))) {}

SharedControlRt::~SharedControlRt() {
	stop();
}

void SharedControlRt::start() {
	running_.store(true, std::memory_order_release);
	thread_ = std::thread([this]() { loop(); });
}

void SharedControlRt::stop() {
	running_.store(false, std::memory_order_release);
	if (thread_.joinable()) {
		thread_.join();
	}
}

void SharedControlRt::loop() {
	best_effort_realtime_setup(90);

	auto next = std::chrono::steady_clock::now();
	while (running_.load(std::memory_order_acquire)) {
		next += period_;

		const InputState in = shared_.inputs.read();
		const OutputState out = compute_safe_command(in);
		shared_.outputs.write(out);

		std::this_thread::sleep_until(next);
	}
}

}  // namespace safety_controller

#ifdef SHARED_CONTROL_RT_STANDALONE
int main() {
	safety_controller::SharedMemory shared;
	safety_controller::SharedControlRt rt(shared, 1000.0);
	rt.start();
	std::this_thread::sleep_for(std::chrono::seconds(2));
	rt.stop();
	return 0;
}
#endif
