#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <functional>

class TeleControllerInput : public rclcpp::Node
{
public:
    TeleControllerInput() : Node("tele_controller_input"), tf_buffer_(this->get_clock())
    {
        // Subscribe to joystick input
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&TeleControllerInput::joy_callback, this, std::placeholders::_1));
        
        // Subscribe to joint states to get current pose for coordinate transforms
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&TeleControllerInput::joint_state_callback, this, std::placeholders::_1));
        
        // Publish velocity commands
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/panda_velocity_cmd", 10);

        // Publish joint trajectories for safe pose
        safe_pose_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10);
        
        // Haptic feedback publishers - CORRECTED
        emergency_pub_ = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);
        mode_change_pub_ = this->create_publisher<std_msgs::msg::Bool>("/teleop_mode_changed", 10);

        // Latched publisher of the active coordinate frame so late-joining nodes
        // (e.g. the status dashboard) always see the current GLOBAL/LOCAL state.
        frame_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/teleop_frame", rclcpp::QoS(1).transient_local());

        // Parameters for scaling
        this->declare_parameter("linear_scale", 0.15);
        this->declare_parameter("angular_scale", 0.3);
        this->declare_parameter("deadzone", 0.15);
        this->declare_parameter("trigger_threshold", 0.5);
        this->declare_parameter("x_button_index", 2);
        this->declare_parameter("velocity_controller", "arm_velocity_controller");
        this->declare_parameter("position_controller", "arm_controller");

        // Hold this button to command orientation (roll/pitch/yaw) instead of
        // translation, giving full 6-DoF teleop. -1 disables orientation mode.
        this->declare_parameter("orientation_button_index", 5);
        // Bounded reconnect for controller switches (handles a controller_manager
        // that drops and comes back).
        this->declare_parameter("switch_max_retries", 3);
        this->declare_parameter("switch_retry_delay", 1.0);

        x_button_index_ = this->get_parameter("x_button_index").as_int();
        velocity_controller_ = this->get_parameter("velocity_controller").as_string();
        position_controller_ = this->get_parameter("position_controller").as_string();
        orientation_button_index_ = this->get_parameter("orientation_button_index").as_int();
        switch_max_retries_ = this->get_parameter("switch_max_retries").as_int();
        switch_retry_delay_ = this->get_parameter("switch_retry_delay").as_double();

        // Frames for local control
        this->declare_parameter("base_frame", "panda_link0");
        this->declare_parameter("tcp_frame", "panda_link7");
        base_frame_ = this->get_parameter("base_frame").as_string();
        tcp_frame_ = this->get_parameter("tcp_frame").as_string();

        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

        switch_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");
        
        RCLCPP_INFO(this->get_logger(), "Teleop controller initialized");
        RCLCPP_INFO(this->get_logger(), "Controls:");
        RCLCPP_INFO(this->get_logger(), "  Left stick: X/Y translation");
        RCLCPP_INFO(this->get_logger(), "  Right stick: Z translation & Z rotation");
        RCLCPP_INFO(this->get_logger(), "  RIGHT TRIGGER: Enable motion (MUST BE PRESSED!)");
        RCLCPP_INFO(this->get_logger(), "  Left trigger: Precision mode");
        RCLCPP_INFO(this->get_logger(), "  A button: Emergency stop");
        RCLCPP_INFO(this->get_logger(), "  B button: Safe joint pose");
        RCLCPP_INFO(this->get_logger(), "  X button: Toggle controller");
        RCLCPP_INFO(this->get_logger(), "  Y button: Toggle coordinate frame (Global/Local)");
        RCLCPP_INFO(this->get_logger(), "  Orientation button (hold): sticks command roll/pitch/yaw (6-DoF)");
        RCLCPP_INFO(this->get_logger(), "  Current frame: GLOBAL (base frame)");

        publishFrame();
    }

    void publishFrame()
    {
        auto msg = std_msgs::msg::String();
        msg.data = use_local_frame_ ? "LOCAL" : "GLOBAL";
        frame_pub_->publish(msg);
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Store latest joint states for coordinate frame calculations
        if (msg->name.size() >= 7 && msg->position.size() >= 7) {
            latest_joint_states_ = *msg;
            has_joint_states_ = true;
        }
    }
    
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Check for B button press (button 1) for safe pose
        if (msg->buttons.size() > 1 && msg->buttons[1] && !previous_b_button_) {
            RCLCPP_INFO(this->get_logger(), "B button pressed - Sending safe joint pose");
            startSafePoseSequence();
            return;
        }
        
        // Check for Y button press (button 3) for coordinate frame toggle
        if (msg->buttons.size() > 3 && msg->buttons[3] && !previous_y_button_) {
            use_local_frame_ = !use_local_frame_;
            std::string frame_name = use_local_frame_ ? "LOCAL (TCP frame)" : "GLOBAL (base frame)";
            RCLCPP_INFO(this->get_logger(), "Y button pressed - Switched to %s", frame_name.c_str());
            publishFrame();

            // Trigger haptic feedback for mode change - CORRECTED
            auto mode_msg = std_msgs::msg::Bool();
            mode_msg.data = true;
            mode_change_pub_->publish(mode_msg);
        }

        // Check for X button press for controller toggle
        if (x_button_index_ >= 0 &&
            msg->buttons.size() > static_cast<size_t>(x_button_index_) &&
            msg->buttons[x_button_index_] && !previous_x_button_) {
            toggleControllerMode();
            return;
        }
        
        // Store previous button states for edge detection
        if (msg->buttons.size() > 1) previous_b_button_ = msg->buttons[1];
        if (msg->buttons.size() > 3) previous_y_button_ = msg->buttons[3];
        if (x_button_index_ >= 0 && msg->buttons.size() > static_cast<size_t>(x_button_index_)) {
            previous_x_button_ = msg->buttons[x_button_index_];
        }
        
        auto twist = geometry_msgs::msg::Twist();
        
        // Get parameters
        double linear_scale = this->get_parameter("linear_scale").as_double();
        double angular_scale = this->get_parameter("angular_scale").as_double();
        double deadzone = this->get_parameter("deadzone").as_double();
        double trigger_threshold = this->get_parameter("trigger_threshold").as_double();
        
        // Check if we have enough axes
        if (msg->axes.size() < 6) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Not enough joystick axes. Expected at least 6, got %zu", msg->axes.size());
            publishZeroVelocity();
            return;
        }
        
        // Enhanced deadzone function
        auto apply_deadzone = [deadzone](double value) {
            if (std::abs(value) < deadzone) {
                return 0.0;
            }
            double sign = (value > 0) ? 1.0 : -1.0;
            return sign * (std::abs(value) - deadzone) / (1.0 - deadzone);
        };
        
        // Get joystick inputs with deadzone
        double left_x = apply_deadzone(msg->axes[0]);
        double left_y = apply_deadzone(msg->axes[1]);
        double right_x = apply_deadzone(msg->axes[3]);
        double right_y = apply_deadzone(msg->axes[4]);
        
        // Get trigger values
        double left_trigger = msg->axes[2];
        double right_trigger = msg->axes[5];
        
        // Convert trigger range from [-1, 1] to [0, 1]
        double left_trigger_pressed = (1.0 - left_trigger) / 2.0;
        double right_trigger_pressed = (1.0 - right_trigger) / 2.0;
        
        // Check emergency stop button (button 0 = A button) - CORRECTED
        if (!msg->buttons.empty() && msg->buttons[0]) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "EMERGENCY STOP ACTIVATED");
            publishZeroVelocity();
            
            // Trigger haptic feedback - CORRECTED
            auto emergency_msg = std_msgs::msg::Bool();
            emergency_msg.data = true;
            emergency_pub_->publish(emergency_msg);
            return;
        }
        
        // RIGHT TRIGGER MUST BE PRESSED TO ENABLE MOTION
        if (right_trigger_pressed < trigger_threshold) {
            publishZeroVelocity();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Motion DISABLED - Press RIGHT TRIGGER to enable (current: %.2f, need: %.2f)",
                right_trigger_pressed, trigger_threshold);
            return;
        }
        
        // Calculate speed multiplier
        double speed_multiplier = 1.0;
        if (left_trigger_pressed > trigger_threshold) {
            speed_multiplier = 0.3;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "PRECISION MODE");
        }
        
        // Orientation mode: while the modifier button is held, the sticks
        // command roll/pitch/yaw (full 6-DoF teleop) instead of translation.
        bool orient_mode = (orientation_button_index_ >= 0 &&
            msg->buttons.size() > static_cast<size_t>(orientation_button_index_) &&
            msg->buttons[orientation_button_index_]);

        // Create base twist command (in joystick reference frame)
        geometry_msgs::msg::Twist base_twist;
        if (orient_mode) {
            base_twist.angular.x = left_y * angular_scale * speed_multiplier;   // Roll
            base_twist.angular.y = -left_x * angular_scale * speed_multiplier;  // Pitch
            base_twist.angular.z = -right_x * angular_scale * speed_multiplier; // Yaw
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "ORIENTATION MODE (roll/pitch/yaw)");
        } else {
            base_twist.linear.x = left_y * linear_scale * speed_multiplier;    // Forward/backward
            base_twist.linear.y = -left_x * linear_scale * speed_multiplier;   // Left/right (inverted)
            base_twist.linear.z = right_y * linear_scale * speed_multiplier;   // Up/down
            base_twist.angular.z = -right_x * angular_scale * speed_multiplier; // Yaw rotation
        }
        
        // Transform twist based on coordinate frame selection
        if (use_local_frame_ && has_joint_states_) {
            twist = transformToLocalFrame(base_twist);
        } else {
            twist = base_twist; // Use global frame (base frame coordinates)
        }
        
        // Publish the velocity command
        velocity_pub_->publish(twist);
        
        // Debug output (throttled)
        if (std::abs(twist.linear.x) > 0.01 ||
            std::abs(twist.linear.y) > 0.01 ||
            std::abs(twist.linear.z) > 0.01 ||
            std::abs(twist.angular.x) > 0.01 ||
            std::abs(twist.angular.y) > 0.01 ||
            std::abs(twist.angular.z) > 0.01) {
            
            std::string frame_str = use_local_frame_ ? "LOCAL" : "GLOBAL";
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "%s: linear=[%.2f, %.2f, %.2f], angular=[%.2f, %.2f, %.2f], speed=%.2f",
                frame_str.c_str(),
                twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z,
                speed_multiplier);
        } else {
            std::string frame_str = use_local_frame_ ? "LOCAL" : "GLOBAL";
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Motion ENABLED (%s frame) - triggers=[L:%.2f, R:%.2f]",
                frame_str.c_str(), left_trigger_pressed, right_trigger_pressed);
        }
    }
    
    geometry_msgs::msg::Twist transformToLocalFrame(const geometry_msgs::msg::Twist& local_twist)
    {
        geometry_msgs::msg::Twist base_twist = local_twist;

        try {
            const auto tf = tf_buffer_.lookupTransform(
                base_frame_, tcp_frame_, tf2::TimePointZero);

            tf2::Quaternion q;
            tf2::fromMsg(tf.transform.rotation, q);
            tf2::Matrix3x3 rotation(q);

            tf2::Vector3 v_local(local_twist.linear.x, local_twist.linear.y, local_twist.linear.z);
            tf2::Vector3 w_local(local_twist.angular.x, local_twist.angular.y, local_twist.angular.z);

            tf2::Vector3 v_base = rotation * v_local;
            tf2::Vector3 w_base = rotation * w_local;

            base_twist.linear.x = v_base.x();
            base_twist.linear.y = v_base.y();
            base_twist.linear.z = v_base.z();
            base_twist.angular.x = w_base.x();
            base_twist.angular.y = w_base.y();
            base_twist.angular.z = w_base.z();
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF lookup failed (%s -> %s): %s",
                base_frame_.c_str(), tcp_frame_.c_str(), ex.what());
        }

        return base_twist;
    }
    
    void publishZeroVelocity()
    {
        auto zero_twist = geometry_msgs::msg::Twist();
        velocity_pub_->publish(zero_twist);
    }

    void publishSafePose()
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = {
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7"
        };

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {0.049, -0.13, -0.19, -1.69, -0.021, 1.55, 0.29};
        point.time_from_start.sec = 2;
        point.time_from_start.nanosec = 0;
        traj.points.push_back(point);

        safe_pose_pub_->publish(traj);
    }

    void startSafePoseSequence()
    {
        if (switch_in_progress_) {
            RCLCPP_WARN(this->get_logger(), "Controller switch in progress");
            return;
        }

        if (!switch_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Switch controller service not available");
            return;
        }

        if (!using_velocity_controller_) {
            publishSafePose();
            return;
        }

        publishZeroVelocity();
        requestControllerSwitch(
            {position_controller_},
            {velocity_controller_},
            [this](bool ok) {
                if (!ok) {
                    RCLCPP_WARN(this->get_logger(), "Failed to switch to position controller");
                    return;
                }

                using_velocity_controller_ = false;
                publishSafePose();

                if (switch_back_timer_) {
                    switch_back_timer_->cancel();
                }

                switch_back_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(2500),
                    [this]() {
                        if (switch_back_timer_) {
                            switch_back_timer_->cancel();
                        }
                        requestControllerSwitch(
                            {velocity_controller_},
                            {position_controller_},
                            [this](bool ok) {
                                if (!ok) {
                                    RCLCPP_WARN(this->get_logger(), "Failed to switch back to velocity controller");
                                    return;
                                }
                                using_velocity_controller_ = true;
                                RCLCPP_INFO(this->get_logger(), "Switched back to velocity controller");
                            });
                    });
            });
    }

    void toggleControllerMode()
    {
        if (switch_in_progress_) {
            RCLCPP_WARN(this->get_logger(), "Controller switch in progress");
            return;
        }

        if (!switch_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Switch controller service not available");
            return;
        }

        if (using_velocity_controller_) {
            publishZeroVelocity();
            requestControllerSwitch(
                {position_controller_},
                {velocity_controller_},
                [this](bool ok) {
                    if (!ok) {
                        RCLCPP_WARN(this->get_logger(), "Controller switch returned ok=false");
                        return;
                    }
                    using_velocity_controller_ = false;
                    RCLCPP_INFO(this->get_logger(), "Switched to position controller");
                });
        } else {
            requestControllerSwitch(
                {velocity_controller_},
                {position_controller_},
                [this](bool ok) {
                    if (!ok) {
                        RCLCPP_WARN(this->get_logger(), "Controller switch returned ok=false");
                        return;
                    }
                    using_velocity_controller_ = true;
                    RCLCPP_INFO(this->get_logger(), "Switched to velocity controller");
                });
        }
    }

    // Request a controller switch, retrying up to switch_max_retries_ times with
    // a fixed delay if the service is unavailable or returns failure. This makes
    // teleop resilient to a controller_manager that briefly drops and reconnects.
    // on_done is invoked once, with the terminal success/failure.
    void requestControllerSwitch(const std::vector<std::string>& activate,
                                 const std::vector<std::string>& deactivate,
                                 std::function<void(bool)> on_done,
                                 int attempts_left = -1)
    {
        if (attempts_left < 0) {
            attempts_left = switch_max_retries_;
        }

        if (!switch_client_->service_is_ready()) {
            if (attempts_left > 0) {
                RCLCPP_WARN(this->get_logger(),
                    "Switch service not ready, retrying in %.1fs (%d attempts left)",
                    switch_retry_delay_, attempts_left);
                scheduleSwitchRetry(activate, deactivate, on_done, attempts_left - 1);
            } else {
                RCLCPP_WARN(this->get_logger(), "Switch service unavailable, giving up");
                if (on_done) on_done(false);
            }
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
        request->activate_controllers = activate;
        request->deactivate_controllers = deactivate;

        switch_in_progress_ = true;
        switch_client_->async_send_request(
            request,
            [this, activate, deactivate, on_done, attempts_left](
                rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
                switch_in_progress_ = false;
                bool ok = false;
                try {
                    ok = future.get()->ok;
                } catch (const std::exception& ex) {
                    RCLCPP_WARN(this->get_logger(), "Controller switch exception: %s", ex.what());
                }

                if (ok) {
                    if (on_done) on_done(true);
                    return;
                }

                if (attempts_left > 0) {
                    RCLCPP_WARN(this->get_logger(),
                        "Controller switch failed, retrying in %.1fs (%d attempts left)",
                        switch_retry_delay_, attempts_left);
                    scheduleSwitchRetry(activate, deactivate, on_done, attempts_left - 1);
                } else if (on_done) {
                    on_done(false);
                }
            });
    }

    // Schedule a single delayed retry of requestControllerSwitch.
    void scheduleSwitchRetry(const std::vector<std::string>& activate,
                             const std::vector<std::string>& deactivate,
                             std::function<void(bool)> on_done,
                             int attempts_left)
    {
        if (retry_timer_) {
            retry_timer_->cancel();
        }
        const auto delay = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(switch_retry_delay_));
        retry_timer_ = this->create_wall_timer(delay,
            [this, activate, deactivate, on_done, attempts_left]() {
                if (retry_timer_) {
                    retry_timer_->cancel();
                }
                requestControllerSwitch(activate, deactivate, on_done, attempts_left);
            });
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr safe_pose_pub_;
    
    // Haptic feedback publishers - CORRECTED
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_change_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr frame_pub_;

    bool previous_b_button_ = false;
    bool previous_x_button_ = false;
    bool previous_y_button_ = false;
    bool use_local_frame_ = false;  // false = global frame, true = local TCP frame
    bool has_joint_states_ = false;
    sensor_msgs::msg::JointState latest_joint_states_;

    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string base_frame_;
    std::string tcp_frame_;

    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
    bool using_velocity_controller_ = true;
    bool switch_in_progress_ = false;
    int x_button_index_ = 2;
    int orientation_button_index_ = 5;
    int switch_max_retries_ = 3;
    double switch_retry_delay_ = 1.0;
    std::string velocity_controller_;
    std::string position_controller_;

    rclcpp::TimerBase::SharedPtr switch_back_timer_;
    rclcpp::TimerBase::SharedPtr retry_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleControllerInput>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}