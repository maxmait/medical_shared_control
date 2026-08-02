// Eyeball curvature-following controller.
//
// Sits between teleop and the safety bridge: it republishes the teleop twist on
// /panda_velocity_cmd_user, and when engaged it reconstructs the eye as a sphere
// from the tool-tip LiDAR sweep, then augments the command to hold a fixed
// stand-off from the curved surface with the probe held perpendicular to it,
// while the user still drives the lateral (tangential) motion.
//
// Being upstream of the safety bridge, all commands still pass through the
// distance-scaling, clamping and watchdog of the existing safety layer.
//
// State machine (cycled by a joystick button):
//   DISABLED  -> pass the teleop twist through unchanged.
//   SCANNING  -> pass teleop through AND collect surface points; auto-advance to
//                FOLLOWING once a good sphere fit lands.
//   FOLLOWING -> command tangential (user) + normal stand-off + reorientation.

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "panda_controller/sphere_fit.hpp"

namespace {

enum class Mode { DISABLED, SCANNING, FOLLOWING };

const char* mode_name(Mode m) {
	switch (m) {
		case Mode::DISABLED: return "DISABLED";
		case Mode::SCANNING: return "SCANNING";
		case Mode::FOLLOWING: return "FOLLOWING";
	}
	return "UNKNOWN";
}

double clamp_abs(double v, double limit) {
	if (v > limit) return limit;
	if (v < -limit) return -limit;
	return v;
}

}  // namespace

class EyeFollowController : public rclcpp::Node {
public:
	EyeFollowController() : Node("eye_follow_controller"), tf_buffer_(get_clock()) {
		base_frame_ = declare_parameter<std::string>("base_frame", "panda_link0");
		lidar_frame_ = declare_parameter<std::string>("lidar_frame", "panda_lidar_frame");
		lidar_axis_ = axisFromString(declare_parameter<std::string>("lidar_axis", "x"));
		follow_button_ = declare_parameter<int>("follow_button_index", 4);

		standoff_ = declare_parameter<double>("standoff", 0.01);
		k_normal_ = declare_parameter<double>("k_normal", 2.0);
		k_orient_ = declare_parameter<double>("k_orient", 2.0);
		min_scan_points_ = declare_parameter<int>("min_scan_points", 40);
		// Bounding-box diagonal of collected points required before fitting. The
		// eye is ~23 mm across, so this must be well under that; ~12 mm (about the
		// radius) gives a well-conditioned fit without needing a full hemisphere.
		min_scan_spread_ = declare_parameter<double>("min_scan_spread", 0.012);
		radius_min_ = declare_parameter<double>("radius_min", 0.005);
		radius_max_ = declare_parameter<double>("radius_max", 0.03);
		max_residual_ = declare_parameter<double>("max_residual", 0.005);
		max_lin_ = declare_parameter<double>("max_lin", 0.1);
		max_ang_ = declare_parameter<double>("max_ang", 1.0);
		const double publish_rate = declare_parameter<double>("publish_rate", 50.0);
		input_timeout_ = declare_parameter<double>("input_timeout", 0.3);

		tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

		teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
			"/panda_velocity_cmd_teleop", 10,
			std::bind(&EyeFollowController::teleopCallback, this, std::placeholders::_1));
		scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
			"/probe/scan", rclcpp::SensorDataQoS(),
			std::bind(&EyeFollowController::scanCallback, this, std::placeholders::_1));
		joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
			"/joy", rclcpp::SensorDataQoS(),
			std::bind(&EyeFollowController::joyCallback, this, std::placeholders::_1));

		cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/panda_velocity_cmd_user", 10);
		state_pub_ = create_publisher<std_msgs::msg::String>(
			"/eye_follow/state", rclcpp::QoS(1).transient_local());

		const auto period = std::chrono::duration<double>(1.0 / publish_rate);
		timer_ = create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(period),
			std::bind(&EyeFollowController::update, this));

		publishState();
		RCLCPP_INFO(get_logger(),
			"Eye-follow controller ready. Button %d cycles DISABLED -> SCANNING -> FOLLOWING.",
			follow_button_);
	}

private:
	// --- inputs ---------------------------------------------------------
	void teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
		last_teleop_ = *msg;
		last_teleop_stamp_ = now();
	}

	void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		// Median-of-up-to-3 valid returns (mirrors shared_control_bridge).
		std::array<double, 3> valid{};
		int count = 0;
		for (double r : msg->ranges) {
			if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
				continue;
			}
			if (count < 3) {
				valid[count++] = r;
			}
		}
		if (count == 0) {
			have_range_ = false;
			return;
		}
		if (count == 1) {
			range_ = valid[0];
		} else if (count == 2) {
			range_ = 0.5 * (valid[0] + valid[1]);
		} else {
			range_ = valid[0] + valid[1] + valid[2]
				- std::min({valid[0], valid[1], valid[2]})
				- std::max({valid[0], valid[1], valid[2]});
		}
		have_range_ = true;
		range_stamp_ = now();
	}

	void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
		const bool pressed = follow_button_ >= 0
			&& static_cast<size_t>(follow_button_) < msg->buttons.size()
			&& msg->buttons[follow_button_] != 0;
		if (pressed && !prev_button_) {
			onButton();
		}
		prev_button_ = pressed;
	}

	void onButton() {
		switch (mode_) {
			case Mode::DISABLED:
				scan_points_.clear();
				setMode(Mode::SCANNING);
				RCLCPP_INFO(get_logger(), "Scanning — sweep the probe across the eye.");
				break;
			case Mode::SCANNING:
			case Mode::FOLLOWING:
				setMode(Mode::DISABLED);
				RCLCPP_INFO(get_logger(), "Eye-follow disengaged.");
				break;
		}
	}

	// --- main loop ------------------------------------------------------
	void update() {
		std::array<double, 3> tip{};
		std::array<double, 3> ray{};
		const bool have_pose = getToolState(tip, ray);

		if (mode_ == Mode::SCANNING && have_pose && rangeFresh()) {
			collectPoint(tip, ray);
			tryFit();
		}

		if (mode_ == Mode::FOLLOWING) {
			if (have_pose) {
				publishFollow(tip, ray);
			}
			return;
		}

		// DISABLED / SCANNING: pass the teleop twist through, but only while it is
		// fresh, so a stalled teleop stream still trips the downstream watchdog.
		if (teleopFresh()) {
			cmd_pub_->publish(last_teleop_);
		}
	}

	void collectPoint(const std::array<double, 3>& tip, const std::array<double, 3>& ray) {
		scan_points_.push_back({tip[0] + range_ * ray[0],
		                        tip[1] + range_ * ray[1],
		                        tip[2] + range_ * ray[2]});
		// Cap memory; keep the most recent points if the sweep runs long.
		if (scan_points_.size() > 2000) {
			scan_points_.erase(scan_points_.begin(),
			                   scan_points_.begin() + 1000);
		}
	}

	void tryFit() {
		if (static_cast<int>(scan_points_.size()) < min_scan_points_) {
			return;
		}
		if (spread(scan_points_) < min_scan_spread_) {
			return;
		}
		const auto fit = panda_controller::fit_sphere(scan_points_);
		if (!fit.ok || fit.radius < radius_min_ || fit.radius > radius_max_
			|| fit.residual > max_residual_) {
			return;  // keep scanning until a clean fit lands
		}
		sphere_center_ = fit.center;
		sphere_radius_ = fit.radius;
		RCLCPP_INFO(get_logger(),
			"Eye reconstructed: center=[%.3f %.3f %.3f] r=%.4f m (residual %.4f, %zu pts). Following.",
			fit.center[0], fit.center[1], fit.center[2], fit.radius, fit.residual,
			scan_points_.size());
		setMode(Mode::FOLLOWING);
	}

	void publishFollow(const std::array<double, 3>& tip, const std::array<double, 3>& ray) {
		geometry_msgs::msg::Twist out;

		// Normal stand-off regulation from the live range (only when we have a
		// valid reading; otherwise hold and let orientation keep us aimed).
		if (rangeFresh()) {
			const double v = k_normal_ * (range_ - standoff_);
			out.linear.x += v * ray[0];
			out.linear.y += v * ray[1];
			out.linear.z += v * ray[2];
		}

		// Reorient so the beam points at the sphere centre (surface normal).
		std::array<double, 3> to_center{sphere_center_[0] - tip[0],
		                                sphere_center_[1] - tip[1],
		                                sphere_center_[2] - tip[2]};
		if (normalize(to_center)) {
			// omega = k * (ray x d_des) rotates the ray toward the desired dir.
			out.angular.x = k_orient_ * (ray[1] * to_center[2] - ray[2] * to_center[1]);
			out.angular.y = k_orient_ * (ray[2] * to_center[0] - ray[0] * to_center[2]);
			out.angular.z = k_orient_ * (ray[0] * to_center[1] - ray[1] * to_center[0]);
		}

		// User tangential drive: project teleop linear velocity off the ray.
		if (teleopFresh()) {
			const std::array<double, 3> vu{last_teleop_.linear.x,
			                               last_teleop_.linear.y,
			                               last_teleop_.linear.z};
			const double along = vu[0] * ray[0] + vu[1] * ray[1] + vu[2] * ray[2];
			out.linear.x += vu[0] - along * ray[0];
			out.linear.y += vu[1] - along * ray[1];
			out.linear.z += vu[2] - along * ray[2];
		}

		out.linear.x = clamp_abs(out.linear.x, max_lin_);
		out.linear.y = clamp_abs(out.linear.y, max_lin_);
		out.linear.z = clamp_abs(out.linear.z, max_lin_);
		out.angular.x = clamp_abs(out.angular.x, max_ang_);
		out.angular.y = clamp_abs(out.angular.y, max_ang_);
		out.angular.z = clamp_abs(out.angular.z, max_ang_);
		cmd_pub_->publish(out);
	}

	// --- helpers --------------------------------------------------------
	bool getToolState(std::array<double, 3>& tip, std::array<double, 3>& ray) {
		try {
			const auto tf = tf_buffer_.lookupTransform(
				base_frame_, lidar_frame_, tf2::TimePointZero);
			tip = {tf.transform.translation.x,
			       tf.transform.translation.y,
			       tf.transform.translation.z};
			tf2::Quaternion q;
			tf2::fromMsg(tf.transform.rotation, q);
			const tf2::Vector3 axis = tf2::Matrix3x3(q) * lidar_axis_;
			ray = {axis.x(), axis.y(), axis.z()};
			return normalize(ray);
		} catch (const tf2::TransformException& ex) {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
				"TF %s <- %s failed: %s",
				base_frame_.c_str(), lidar_frame_.c_str(), ex.what());
			return false;
		}
	}

	static bool normalize(std::array<double, 3>& v) {
		const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
		if (n < 1e-9) return false;
		v[0] /= n; v[1] /= n; v[2] /= n;
		return true;
	}

	static double spread(const std::vector<std::array<double, 3>>& pts) {
		std::array<double, 3> lo{pts[0]};
		std::array<double, 3> hi{pts[0]};
		for (const auto& p : pts) {
			for (int k = 0; k < 3; ++k) {
				lo[k] = std::min(lo[k], p[k]);
				hi[k] = std::max(hi[k], p[k]);
			}
		}
		const double dx = hi[0] - lo[0], dy = hi[1] - lo[1], dz = hi[2] - lo[2];
		return std::sqrt(dx * dx + dy * dy + dz * dz);
	}

	tf2::Vector3 axisFromString(const std::string& a) const {
		if (a == "x") return {1, 0, 0};
		if (a == "-x") return {-1, 0, 0};
		if (a == "y") return {0, 1, 0};
		if (a == "-y") return {0, -1, 0};
		if (a == "z") return {0, 0, 1};
		if (a == "-z") return {0, 0, -1};
		return {1, 0, 0};
	}

	bool teleopFresh() const {
		return last_teleop_stamp_.nanoseconds() > 0
			&& (now() - last_teleop_stamp_).seconds() < input_timeout_;
	}
	bool rangeFresh() const {
		return have_range_ && (now() - range_stamp_).seconds() < input_timeout_;
	}

	void setMode(Mode m) {
		if (m != mode_) {
			mode_ = m;
			publishState();
		}
	}
	void publishState() {
		std_msgs::msg::String s;
		s.data = mode_name(mode_);
		if (mode_ == Mode::FOLLOWING) {
			char buf[64];
			std::snprintf(buf, sizeof(buf), "FOLLOWING (r=%.1f mm)", sphere_radius_ * 1000.0);
			s.data = buf;
		}
		state_pub_->publish(s);
	}

	// --- members --------------------------------------------------------
	std::string base_frame_, lidar_frame_;
	tf2::Vector3 lidar_axis_{1, 0, 0};
	int follow_button_ = 4;
	double standoff_, k_normal_, k_orient_, min_scan_spread_;
	double radius_min_, radius_max_, max_residual_, max_lin_, max_ang_, input_timeout_;
	int min_scan_points_ = 40;

	tf2_ros::Buffer tf_buffer_;
	std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
	rclcpp::TimerBase::SharedPtr timer_;

	Mode mode_ = Mode::DISABLED;
	bool prev_button_ = false;

	geometry_msgs::msg::Twist last_teleop_;
	rclcpp::Time last_teleop_stamp_{0, 0, RCL_ROS_TIME};

	double range_ = 0.0;
	bool have_range_ = false;
	rclcpp::Time range_stamp_{0, 0, RCL_ROS_TIME};

	std::vector<std::array<double, 3>> scan_points_;
	std::array<double, 3> sphere_center_{0, 0, 0};
	double sphere_radius_ = 0.0;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EyeFollowController>());
	rclcpp::shutdown();
	return 0;
}
