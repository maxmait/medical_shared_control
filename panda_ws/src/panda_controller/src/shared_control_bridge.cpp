#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "shared_memory.hpp"
#include "shared_control_rt.hpp"

class SharedControlBridge : public rclcpp::Node {
public:
		SharedControlBridge()
				: Node("shared_control_bridge"),
					rt_(shared_, 1000.0),
					tf_buffer_(this->get_clock()) {
		declare_parameter<std::string>("lidar_topic", "/probe/scan");
		declare_parameter<std::string>("user_cmd_topic", "/panda_velocity_cmd_user");
		declare_parameter<std::string>("joint_state_topic", "/joint_states");
		declare_parameter<std::string>("output_cmd_topic", "/panda_velocity_cmd");
		declare_parameter<double>("publish_rate", 100.0);
		declare_parameter<std::string>("base_frame", "panda_link0");
		declare_parameter<std::string>("lidar_frame", "panda_lidar_frame");
		declare_parameter<std::string>("lidar_axis", "x");

		const auto lidar_topic = get_parameter("lidar_topic").as_string();
		const auto user_cmd_topic = get_parameter("user_cmd_topic").as_string();
		const auto joint_state_topic = get_parameter("joint_state_topic").as_string();
		const auto output_cmd_topic = get_parameter("output_cmd_topic").as_string();
		const double publish_rate = get_parameter("publish_rate").as_double();
		base_frame_ = get_parameter("base_frame").as_string();
		lidar_frame_ = get_parameter("lidar_frame").as_string();
		lidar_axis_ = axisFromString(get_parameter("lidar_axis").as_string());

		tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

		lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
			lidar_topic, rclcpp::SensorDataQoS(),
			std::bind(&SharedControlBridge::lidarCallback, this, std::placeholders::_1));

		user_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
			user_cmd_topic, 10,
			std::bind(&SharedControlBridge::userCmdCallback, this, std::placeholders::_1));

		joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
			joint_state_topic, rclcpp::SensorDataQoS(),
			std::bind(&SharedControlBridge::jointStateCallback, this, std::placeholders::_1));

		output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_topic, 10);

		const auto period = std::chrono::duration<double>(1.0 / publish_rate);
		output_timer_ = create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(period),
			std::bind(&SharedControlBridge::publishOutput, this));

		rt_.start();
		RCLCPP_INFO(get_logger(), "Shared control bridge started");
	}

	~SharedControlBridge() override {
		rt_.stop();
	}

private:
	void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		if (msg->ranges.empty()) {
			return;
		}

		std::array<double, 3> valid_ranges{};
		int valid_count = 0;
		for (double r : msg->ranges) {
			if (!std::isfinite(r)) {
				continue;
			}
			if (r < msg->range_min || r > msg->range_max) {
				continue;
			}
			if (valid_count < static_cast<int>(valid_ranges.size())) {
				valid_ranges[valid_count] = r;
				++valid_count;
			}
		}

		double selected_range = msg->range_max;
		if (valid_count == 1) {
			selected_range = valid_ranges[0];
		} else if (valid_count == 2) {
			selected_range = 0.5 * (valid_ranges[0] + valid_ranges[1]);
		} else if (valid_count >= 3) {
			const double a = valid_ranges[0];
			const double b = valid_ranges[1];
			const double c = valid_ranges[2];
			selected_range = a + b + c - std::min(a, std::min(b, c)) - std::max(a, std::max(b, c));
		}

		input_cache_.lidar_range = selected_range;
		updateLidarDirection();
		input_cache_.has_lidar_dir = has_lidar_dir_;
		input_cache_.lidar_dir_base[0] = lidar_dir_base_[0];
		input_cache_.lidar_dir_base[1] = lidar_dir_base_[1];
		input_cache_.lidar_dir_base[2] = lidar_dir_base_[2];
		shared_.inputs.write(input_cache_);
	}

	void userCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
		input_cache_.cmd_linear[0] = msg->linear.x;
		input_cache_.cmd_linear[1] = msg->linear.y;
		input_cache_.cmd_linear[2] = msg->linear.z;
		input_cache_.cmd_angular[0] = msg->angular.x;
		input_cache_.cmd_angular[1] = msg->angular.y;
		input_cache_.cmd_angular[2] = msg->angular.z;
		shared_.inputs.write(input_cache_);
	}

	void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
		if (msg->name.size() != msg->position.size()) {
			return;
		}

		for (size_t i = 0; i < msg->name.size(); ++i) {
			const auto& name = msg->name[i];
			if (name.find("panda_joint") != std::string::npos && !name.empty()) {
				const char last = name.back();
				if (last >= '1' && last <= '7') {
					const size_t joint_index = static_cast<size_t>(last - '1');
					if (joint_index < 7) {
						input_cache_.joint_pos[joint_index] = msg->position[i];
						input_cache_.has_joint_state = true;
					}
				}
			}
		}

		shared_.inputs.write(input_cache_);
	}

	void publishOutput() {
		const auto out = shared_.outputs.read();

		output_msg_.linear.x = out.safe_cmd_linear[0];
		output_msg_.linear.y = out.safe_cmd_linear[1];
		output_msg_.linear.z = out.safe_cmd_linear[2];
		output_msg_.angular.x = out.safe_cmd_angular[0];
		output_msg_.angular.y = out.safe_cmd_angular[1];
		output_msg_.angular.z = out.safe_cmd_angular[2];

		output_pub_->publish(output_msg_);
	}

	tf2::Vector3 axisFromString(const std::string& axis) const {
		if (axis == "x") {
			return tf2::Vector3(1.0, 0.0, 0.0);
		}
		if (axis == "-x") {
			return tf2::Vector3(-1.0, 0.0, 0.0);
		}
		if (axis == "y") {
			return tf2::Vector3(0.0, 1.0, 0.0);
		}
		if (axis == "-y") {
			return tf2::Vector3(0.0, -1.0, 0.0);
		}
		if (axis == "-z") {
			return tf2::Vector3(0.0, 0.0, -1.0);
		}
		return tf2::Vector3(0.0, 0.0, 1.0);
	}

	void updateLidarDirection() {
		try {
			const auto tf = tf_buffer_.lookupTransform(
				base_frame_, lidar_frame_, tf2::TimePointZero);

			tf2::Quaternion q;
			tf2::fromMsg(tf.transform.rotation, q);
			tf2::Matrix3x3 rotation(q);

			tf2::Vector3 axis_base = rotation * lidar_axis_;
			if (axis_base.length2() > 1e-12) {
				axis_base.normalize();
			}

			lidar_dir_base_[0] = axis_base.x();
			lidar_dir_base_[1] = axis_base.y();
			lidar_dir_base_[2] = axis_base.z();
			has_lidar_dir_ = true;
		} catch (const tf2::TransformException& ex) {
			RCLCPP_WARN_THROTTLE(
				get_logger(), *get_clock(), 2000,
				"TF lookup failed (%s -> %s): %s",
				base_frame_.c_str(), lidar_frame_.c_str(), ex.what());
			has_lidar_dir_ = false;
		}
	}

	safety_controller::SharedMemory shared_;
	safety_controller::SharedControlRt rt_;
	safety_controller::InputState input_cache_{};

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr user_cmd_sub_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
	rclcpp::TimerBase::SharedPtr output_timer_;

	geometry_msgs::msg::Twist output_msg_{};

	tf2_ros::Buffer tf_buffer_;
	std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
	std::string base_frame_;
	std::string lidar_frame_;
	tf2::Vector3 lidar_axis_{0.0, 0.0, 1.0};
	double lidar_dir_base_[3] = {0.0, 0.0, 1.0};
	bool has_lidar_dir_ = false;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SharedControlBridge>());
	rclcpp::shutdown();
	return 0;
}
