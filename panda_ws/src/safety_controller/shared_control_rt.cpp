#include "shared_control_rt.hpp"

#include <cmath>
#include <cstdint>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

namespace safety_controller {

namespace {

void best_effort_realtime_setup(int priority) {
	if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
		// Non-fatal: without locked memory the RT loop may page-fault, but it
		// still runs. Nothing to log from here (no ROS); the bridge warns.
	}

	sched_param params;
	params.sched_priority = priority;
	if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &params) != 0) {
		// Non-fatal: falls back to the default scheduling policy when the
		// process lacks CAP_SYS_NICE / RT privileges.
	}
}

double clamp_abs(double value, double limit) {
	if (value > limit) return limit;
	if (value < -limit) return -limit;
	return value;
}

}  // namespace

OutputState compute_safe_command(const InputState& in,
                                 const SafetyParams& p,
                                 SafetyState& state,
                                 double input_age_s) {
	OutputState out;

	// Dead-man watchdog: if the inputs have gone stale (a stalled LiDAR or a
	// dropped teleop stream), hold zero output rather than replaying the last
	// command. `out` is zero-initialised, so an early return is a safe stop.
	if (input_age_s < 0.0 || input_age_s > p.input_timeout_s) {
		return out;
	}

	const double stop_distance = p.stop_distance;
	const double slow_distance = p.slow_distance;
	const double release_margin = p.release_margin;
	const double min_scale = p.min_scale;
	const double filter_alpha = p.filter_alpha;
	const double lateral_weight = p.lateral_weight;

	if (!state.range_initialized) {
		state.filtered_range = in.lidar_range;
		state.range_initialized = true;
	} else {
		state.filtered_range =
			(filter_alpha * in.lidar_range) + ((1.0 - filter_alpha) * state.filtered_range);
	}
	const double filtered_range = state.filtered_range;
	double scale = 1.0;

	if (state.stopped) {
		if (filtered_range >= stop_distance + release_margin) {
			state.stopped = false;
		} else {
			scale = min_scale;
		}
	}

	if (!state.stopped && filtered_range <= stop_distance) {
		state.stopped = true;
		scale = min_scale;
	} else if (!state.stopped && filtered_range < slow_distance) {
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

	// Absolute saturation: never emit a command beyond the configured limits,
	// regardless of the incoming command or the scaling above.
	for (int i = 0; i < 3; ++i) {
		out.safe_cmd_linear[i] = clamp_abs(out.safe_cmd_linear[i], p.max_linear_vel);
		out.safe_cmd_angular[i] = clamp_abs(out.safe_cmd_angular[i], p.max_angular_vel);
	}

	return out;
}

SharedControlRt::SharedControlRt(SharedMemory& shared, double rate_hz, const SafetyParams& params)
	: shared_(shared),
	  period_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
		  std::chrono::duration<double>(1.0 / rate_hz))) {
	params_.write(params);
}

void SharedControlRt::set_params(const SafetyParams& params) {
	params_.write(params);
}

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
		const SafetyParams params = params_.read();

		// Input age from the shared steady_clock (same process as the bridge).
		// A never-written input (stamp_ns == 0) reads as maximally stale.
		double input_age_s = params.input_timeout_s + 1.0;
		if (in.stamp_ns != 0) {
			const uint64_t now_ns = static_cast<uint64_t>(
				std::chrono::duration_cast<std::chrono::nanoseconds>(
					std::chrono::steady_clock::now().time_since_epoch()).count());
			if (now_ns >= in.stamp_ns) {
				input_age_s = static_cast<double>(now_ns - in.stamp_ns) * 1e-9;
			} else {
				input_age_s = 0.0;
			}
		}

		const OutputState out = compute_safe_command(in, params, state_, input_age_s);
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
