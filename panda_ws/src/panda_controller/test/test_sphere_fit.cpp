// Unit tests for the pure sphere fit used to reconstruct the eye.

#include "panda_controller/sphere_fit.hpp"

#include <array>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

using panda_controller::fit_sphere;

namespace {

// Sample points on a wide spherical cap (like a LiDAR sweep across the eye),
// with optional symmetric jitter from a fixed sequence so tests are
// deterministic. A reasonably wide cap is needed for a well-conditioned fit.
std::vector<std::array<double, 3>> sample_sphere(
	const std::array<double, 3>& c, double r, double jitter = 0.0) {
	std::vector<std::array<double, 3>> pts;
	int seq = 1;
	auto rnd = [&seq]() {  // deterministic pseudo-noise in [-1, 1]
		const double n = std::sin(seq++ * 12.9898) * 43758.5453;
		return 2.0 * (n - std::floor(n)) - 1.0;
	};
	for (int i = 0; i < 10; ++i) {
		for (int j = 0; j < 10; ++j) {
			const double theta = 0.2 + 0.11 * i;   // polar 0.2..1.19 rad (~68 deg cap)
			const double phi = -0.7 + 0.155 * j;   // azimuth -0.7..0.7
			double x = r * std::sin(theta) * std::cos(phi);
			double y = r * std::sin(theta) * std::sin(phi);
			double z = r * std::cos(theta);
			if (jitter > 0.0) {
				x += jitter * rnd();
				y += jitter * rnd();
				z += jitter * rnd();
			}
			pts.push_back({c[0] + x, c[1] + y, c[2] + z});
		}
	}
	return pts;
}

}  // namespace

TEST(SphereFit, RecoversExactSphere) {
	const std::array<double, 3> c{0.20, 0.52, 0.73};  // eye ground truth
	const double r = 0.0115;
	const auto pts = sample_sphere(c, r);

	const auto fit = fit_sphere(pts);
	ASSERT_TRUE(fit.ok);
	EXPECT_NEAR(fit.center[0], c[0], 1e-6);
	EXPECT_NEAR(fit.center[1], c[1], 1e-6);
	EXPECT_NEAR(fit.center[2], c[2], 1e-6);
	EXPECT_NEAR(fit.radius, r, 1e-6);
	EXPECT_LT(fit.residual, 1e-6);
}

TEST(SphereFit, RobustToNoise) {
	const std::array<double, 3> c{0.20, 0.52, 0.73};
	const double r = 0.0115;
	const auto pts = sample_sphere(c, r, /*jitter=*/0.0003);  // ~lidar resolution

	const auto fit = fit_sphere(pts);
	ASSERT_TRUE(fit.ok);
	EXPECT_NEAR(fit.center[0], c[0], 3e-3);
	EXPECT_NEAR(fit.center[1], c[1], 3e-3);
	EXPECT_NEAR(fit.center[2], c[2], 3e-3);
	EXPECT_NEAR(fit.radius, r, 3e-3);
}

TEST(SphereFit, RejectsTooFewPoints) {
	std::vector<std::array<double, 3>> pts{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
	EXPECT_FALSE(fit_sphere(pts).ok);
}

int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
