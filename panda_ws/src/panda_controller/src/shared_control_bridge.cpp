#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <array>
#include <cmath>
#include <string>

#include "shared_memory.hpp"
#include "shared_control_rt.hpp"

class SharedControlBridge : public rclcpp::Node {
public:
	SharedControlBridge()
		: Node("shared_control_bridge"),
		  rt_(shared_, 1000.0) {
		declare_parameter<std::string>("lidar_topic", "/probe/scan");
		declare_parameter<std::string>("user_cmd_topic", "/panda_velocity_cmd_user");
		declare_parameter<std::string>("joint_state_topic", "/joint_states");
		declare_parameter<std::string>("output_cmd_topic", "/panda_velocity_cmd");
		declare_parameter<double>("publish_rate", 100.0);

		const auto lidar_topic = get_parameter("lidar_topic").as_string();
		const auto user_cmd_topic = get_parameter("user_cmd_topic").as_string();
		const auto joint_state_topic = get_parameter("joint_state_topic").as_string();
		const auto output_cmd_topic = get_parameter("output_cmd_topic").as_string();
		const double publish_rate = get_parameter("publish_rate").as_double();

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

	safety_controller::SharedMemory shared_;
	safety_controller::SharedControlRt rt_;
	safety_controller::InputState input_cache_{};

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr user_cmd_sub_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
	rclcpp::TimerBase::SharedPtr output_timer_;

	geometry_msgs::msg::Twist output_msg_{};
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SharedControlBridge>());
	rclcpp::shutdown();
	return 0;
}
