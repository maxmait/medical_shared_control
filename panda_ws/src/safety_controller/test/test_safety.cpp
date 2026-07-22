// Unit tests for the real-time safety core. These exercise the pure
// compute_safe_command() law and the lock-free LatestValueBuffer directly,
// with no ROS involved.

#include "shared_control_rt.hpp"
#include "shared_memory.hpp"

#include <cmath>
#include <thread>

#include <gtest/gtest.h>

using safety_controller::compute_safe_command;
using safety_controller::InputState;
using safety_controller::LatestValueBuffer;
using safety_controller::OutputState;
using safety_controller::SafetyParams;
using safety_controller::SafetyState;

namespace {

// A params set with an instantaneous range filter (alpha = 1) so tests are
// deterministic on a single call.
SafetyParams instant_filter_params() {
	SafetyParams p;
	p.filter_alpha = 1.0;
	return p;
}

double magnitude(const double v[3]) {
	return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

}  // namespace

// Far from any obstacle, the command passes through unchanged (scale == 1).
TEST(ComputeSafeCommand, PassthroughWhenFar) {
	SafetyParams p = instant_filter_params();
	SafetyState s;
	InputState in;
	in.lidar_range = 1.0;  // well beyond slow_distance
	in.cmd_linear[0] = 0.1;

	OutputState out = compute_safe_command(in, p, s, 0.0);
	EXPECT_NEAR(out.safe_cmd_linear[0], 0.1, 1e-9);
}

// Between stop and slow distance the scale ramps linearly.
TEST(ComputeSafeCommand, LinearRampBetweenStopAndSlow) {
	SafetyParams p = instant_filter_params();
	SafetyState s;
	InputState in;
	in.lidar_range = 0.5 * (p.stop_distance + p.slow_distance);  // midpoint -> scale ~0.5
	in.cmd_linear[0] = 0.1;

	OutputState out = compute_safe_command(in, p, s, 0.0);
	EXPECT_NEAR(out.safe_cmd_linear[0], 0.05, 1e-3);
}

// The stop latch engages below stop_distance and only releases once the range
// clears stop_distance + release_margin (hysteresis).
TEST(ComputeSafeCommand, StopLatchHysteresis) {
	SafetyParams p = instant_filter_params();
	SafetyState s;
	InputState in;
	in.cmd_linear[0] = 0.1;

	// Engage the stop below stop_distance.
	in.lidar_range = p.stop_distance * 0.5;
	OutputState out = compute_safe_command(in, p, s, 0.0);
	EXPECT_TRUE(s.stopped);
	EXPECT_NEAR(out.safe_cmd_linear[0], 0.1 * p.min_scale, 1e-9);

	// Range recovers past stop_distance but not past the release margin: still latched.
	in.lidar_range = p.stop_distance + 0.5 * p.release_margin;
	out = compute_safe_command(in, p, s, 0.0);
	EXPECT_TRUE(s.stopped);
	EXPECT_NEAR(out.safe_cmd_linear[0], 0.1 * p.min_scale, 1e-9);

	// Range clears the release margin: latch releases.
	in.lidar_range = p.stop_distance + p.release_margin + 1e-4;
	out = compute_safe_command(in, p, s, 0.0);
	EXPECT_FALSE(s.stopped);
}

// Direction-aware slowdown: approaching the obstacle is slowed more than
// retreating from it, and lateral motion is only mildly slowed.
TEST(ComputeSafeCommand, DirectionAwareSlowdown) {
	SafetyParams p = instant_filter_params();
	InputState base;
	base.lidar_range = 0.5 * (p.stop_distance + p.slow_distance);  // in the slowdown zone
	base.has_lidar_dir = true;
	base.lidar_dir_base[0] = 1.0;
	base.lidar_dir_base[1] = 0.0;
	base.lidar_dir_base[2] = 0.0;

	// Approach: +x toward the obstacle.
	InputState approach = base;
	approach.cmd_linear[0] = 0.1;
	SafetyState sa;
	OutputState out_app = compute_safe_command(approach, p, sa, 0.0);

	// Retreat: -x away from the obstacle.
	InputState retreat = base;
	retreat.cmd_linear[0] = -0.1;
	SafetyState sr;
	OutputState out_ret = compute_safe_command(retreat, p, sr, 0.0);

	// Lateral: +y, perpendicular to the beam.
	InputState lateral = base;
	lateral.cmd_linear[1] = 0.1;
	SafetyState sl;
	OutputState out_lat = compute_safe_command(lateral, p, sl, 0.0);

	// Retreating and lateral motion keep more speed than approaching.
	EXPECT_GT(magnitude(out_ret.safe_cmd_linear), magnitude(out_app.safe_cmd_linear));
	EXPECT_GT(magnitude(out_lat.safe_cmd_linear), magnitude(out_app.safe_cmd_linear));
}

// Output is clamped to the configured absolute limits.
TEST(ComputeSafeCommand, ClampsToLimits) {
	SafetyParams p = instant_filter_params();
	p.max_linear_vel = 0.25;
	p.max_angular_vel = 1.0;
	SafetyState s;
	InputState in;
	in.lidar_range = 1.0;  // far -> scale 1, no distance slowdown
	in.cmd_linear[0] = 10.0;
	in.cmd_angular[2] = 10.0;

	OutputState out = compute_safe_command(in, p, s, 0.0);
	EXPECT_NEAR(out.safe_cmd_linear[0], 0.25, 1e-9);
	EXPECT_NEAR(out.safe_cmd_angular[2], 1.0, 1e-9);
}

// Watchdog: a stale input (older than input_timeout_s) forces zero output.
TEST(ComputeSafeCommand, WatchdogZerosOnStaleInput) {
	SafetyParams p = instant_filter_params();
	p.input_timeout_s = 0.2;
	SafetyState s;
	InputState in;
	in.lidar_range = 1.0;
	in.cmd_linear[0] = 0.1;
	in.cmd_angular[2] = 0.1;

	OutputState out = compute_safe_command(in, p, s, /*input_age_s=*/0.5);
	EXPECT_EQ(out.safe_cmd_linear[0], 0.0);
	EXPECT_EQ(out.safe_cmd_angular[2], 0.0);

	// A fresh input of the same command is not zeroed.
	OutputState fresh = compute_safe_command(in, p, s, 0.0);
	EXPECT_GT(std::abs(fresh.safe_cmd_linear[0]), 0.0);
}

// The lock-free buffer returns the last value written.
TEST(LatestValueBuffer, RoundTrip) {
	LatestValueBuffer<InputState> buf;
	InputState in;
	in.lidar_range = 0.42;
	in.cmd_linear[1] = -0.3;
	in.stamp_ns = 12345;
	buf.write(in);

	InputState out = buf.read();
	EXPECT_NEAR(out.lidar_range, 0.42, 1e-12);
	EXPECT_NEAR(out.cmd_linear[1], -0.3, 1e-12);
	EXPECT_EQ(out.stamp_ns, 12345u);
}

// The buffer stays consistent under a concurrent writer (no torn reads).
TEST(LatestValueBuffer, ConcurrentReadsAreConsistent) {
	LatestValueBuffer<InputState> buf;
	std::atomic<bool> run{true};

	std::thread writer([&]() {
		double v = 0.0;
		while (run.load()) {
			InputState in;
			in.lidar_range = v;
			in.cmd_linear[0] = v;
			in.cmd_linear[1] = v;
			buf.write(in);
			v += 1.0;
		}
	});

	for (int i = 0; i < 100000; ++i) {
		InputState out = buf.read();
		// All three fields are written from the same v, so a consistent snapshot
		// has them equal. A torn read would break this invariant.
		EXPECT_EQ(out.lidar_range, out.cmd_linear[0]);
		EXPECT_EQ(out.cmd_linear[0], out.cmd_linear[1]);
	}

	run.store(false);
	writer.join();
}

int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
