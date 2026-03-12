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

	for (int i = 0; i < 3; ++i) {
		out.safe_cmd_linear[i] = in.cmd_linear[i] * scale;
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
