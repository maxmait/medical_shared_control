#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

class PandaVelocityController : public rclcpp::Node
{
public:
    PandaVelocityController() : Node("panda_velocity_controller")
    {
        // Initialize KDL chain
        initializeKDL();
        
        // Only create subscribers and publishers if KDL initialization succeeded
        if (kdl_initialized_) {
            // Subscribers
            joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, 
                std::bind(&PandaVelocityController::jointStateCallback, this, std::placeholders::_1));
                
            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/panda_velocity_cmd", 10,
                std::bind(&PandaVelocityController::cmdVelCallback, this, std::placeholders::_1));
            
            // Publisher for joint velocities
            joint_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/arm_velocity_controller/commands", 10);
                
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(20), // 50Hz
                std::bind(&PandaVelocityController::controlLoop, this));
                
            RCLCPP_INFO(this->get_logger(), "Panda velocity controller initialized successfully!");
        }
    }

private:
    void initializeKDL()
    {
        // Get robot description from robot_state_publisher service
        auto client = this->create_client<rcl_interfaces::srv::GetParameters>("/robot_state_publisher/get_parameters");
        
        // Wait for service with timeout
        RCLCPP_INFO(this->get_logger(), "Waiting for robot_state_publisher service...");
        if (!client->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "robot_state_publisher service not available after 10 seconds");
            return;
        }
        
        // Create request
        auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
        request->names.push_back("robot_description");
        
        // Send request and wait for response
        auto result_future = client->async_send_request(request);
        
        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, std::chrono::seconds(5)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service get_parameters");
            return;
        }
        
        auto result = result_future.get();
        
        // Check if we got a response
        if (result->values.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No parameters returned from robot_state_publisher");
            return;
        }
        
        // Check parameter type and get string value
        const auto& param_value = result->values[0];
        if (param_value.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
            RCLCPP_ERROR(this->get_logger(), "robot_description parameter is not a string (type: %d)", param_value.type);
            return;
        }
        
        std::string robot_description = param_value.string_value;
        
        if (robot_description.empty()) {
            RCLCPP_ERROR(this->get_logger(), "robot_description parameter is empty");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully retrieved robot description (%zu characters)", robot_description.length());
        
        // Parse URDF and create KDL tree
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromString(robot_description, kdl_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree from robot description");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "KDL tree constructed successfully");
        
        // Extract chain from base to end-effector
        if (!kdl_tree.getChain("panda_link0", "panda_link7", kdl_chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from panda_link0 to panda_link7");
            
            // Debug: print available segments
            RCLCPP_INFO(this->get_logger(), "Available segments in KDL tree:");
            auto segments = kdl_tree.getSegments();
            for (const auto& segment : segments) {
                RCLCPP_INFO(this->get_logger(), "  - %s", segment.first.c_str());
            }
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "KDL chain extracted successfully");
        
        // Initialize solvers
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
        jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
        
        // Initialize joint arrays
        joint_positions_.resize(kdl_chain_.getNrOfJoints());
        joint_velocities_.resize(kdl_chain_.getNrOfJoints());
        
        // Initialize to zero
        for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
            joint_positions_(i) = 0.0;
            joint_velocities_(i) = 0.0;
        }
        
        kdl_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "KDL chain initialized with %d joints", kdl_chain_.getNrOfJoints());
    }
    
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!kdl_initialized_) return;
        
        // Update joint positions (find the correct mapping)
        for (size_t i = 0; i < msg->name.size() && i < static_cast<size_t>(joint_positions_.rows()); ++i) {
            // Check if this is a panda joint
            if (msg->name[i].find("panda_joint") != std::string::npos) {
                // Extract joint number (assuming format "panda_jointX")
                size_t joint_num = msg->name[i].back() - '1'; // Convert '1'-'7' to 0-6
                if (joint_num < static_cast<size_t>(joint_positions_.rows())) {
                    joint_positions_(joint_num) = msg->position[i];
                }
            }
        }
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_twist_ = *msg;
    }
    
    void controlLoop()
    {
        if (!kdl_initialized_ || !fk_solver_ || !jac_solver_) return;
        
        // Compute Jacobian
        KDL::Jacobian jacobian(kdl_chain_.getNrOfJoints());
        int jac_result = jac_solver_->JntToJac(joint_positions_, jacobian);
        
        if (jac_result != 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to compute Jacobian");
            return;
        }
        
        // Convert Twist to KDL Vector
        KDL::Twist target_vel;
        target_vel.vel.x(target_twist_.linear.x);
        target_vel.vel.y(target_twist_.linear.y);
        target_vel.vel.z(target_twist_.linear.z);
        target_vel.rot.x(target_twist_.angular.x);
        target_vel.rot.y(target_twist_.angular.y);
        target_vel.rot.z(target_twist_.angular.z);
        
        // Compute joint velocities using pseudo-inverse
        KDL::ChainIkSolverVel_pinv ik_vel_solver(kdl_chain_);
        int ik_result = ik_vel_solver.CartToJnt(joint_positions_, target_vel, joint_velocities_);
        
        if (ik_result < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "IK velocity solver failed");
            return;
        }
        
        // Publish joint velocities
        auto vel_msg = std_msgs::msg::Float64MultiArray();
        vel_msg.data.resize(joint_velocities_.rows());
        for (unsigned int i = 0; i < joint_velocities_.rows(); ++i) {
            vel_msg.data[i] = joint_velocities_(i);
        }
        joint_vel_pub_->publish(vel_msg);
    }
    
    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    KDL::Chain kdl_chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    geometry_msgs::msg::Twist target_twist_;
    
    bool kdl_initialized_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PandaVelocityController>());
    rclcpp::shutdown();
    return 0;
}