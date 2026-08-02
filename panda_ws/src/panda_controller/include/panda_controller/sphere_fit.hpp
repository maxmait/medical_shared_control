// Pure least-squares sphere fit — no ROS, so it can be unit-tested directly.
//
// Used by eye_follow_controller to reconstruct the eye as a sphere from the
// tool-tip LiDAR surface points collected during the scan sweep.
#pragma once

#include <array>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace panda_controller {

struct SphereFit {
	std::array<double, 3> center{0.0, 0.0, 0.0};
	double radius = 0.0;
	double residual = 0.0;  // RMS of |dist(point, center) - radius|
	bool ok = false;
};

// Coope's algebraic least-squares sphere fit. For points p_i on a sphere,
//   |p_i - c|^2 = r^2  ->  2 c.p_i + (r^2 - |c|^2) = |p_i|^2,
// which is linear in [c_x, c_y, c_z, f] with f = r^2 - |c|^2. Solve the
// overdetermined system and recover r = sqrt(f + |c|^2).
//
// Requires at least 4 non-coplanar points; returns ok=false otherwise or if the
// solution is degenerate (non-positive radius^2).
inline SphereFit fit_sphere(const std::vector<std::array<double, 3>>& points) {
	SphereFit out;
	const std::size_t n = points.size();
	if (n < 4) {
		return out;
	}

	Eigen::MatrixXd a(static_cast<Eigen::Index>(n), 4);
	Eigen::VectorXd b(static_cast<Eigen::Index>(n));
	for (std::size_t i = 0; i < n; ++i) {
		const double x = points[i][0];
		const double y = points[i][1];
		const double z = points[i][2];
		const auto row = static_cast<Eigen::Index>(i);
		a(row, 0) = 2.0 * x;
		a(row, 1) = 2.0 * y;
		a(row, 2) = 2.0 * z;
		a(row, 3) = 1.0;
		b(row) = (x * x) + (y * y) + (z * z);
	}

	const Eigen::Vector4d u = a.colPivHouseholderQr().solve(b);
	const double cx = u(0);
	const double cy = u(1);
	const double cz = u(2);
	const double r2 = u(3) + (cx * cx) + (cy * cy) + (cz * cz);
	if (!(r2 > 0.0)) {
		return out;
	}

	out.center = {cx, cy, cz};
	out.radius = std::sqrt(r2);

	double acc = 0.0;
	for (std::size_t i = 0; i < n; ++i) {
		const double dx = points[i][0] - cx;
		const double dy = points[i][1] - cy;
		const double dz = points[i][2] - cz;
		const double e = std::sqrt((dx * dx) + (dy * dy) + (dz * dz)) - out.radius;
		acc += e * e;
	}
	out.residual = std::sqrt(acc / static_cast<double>(n));
	out.ok = true;
	return out;
}

}  // namespace panda_controller
