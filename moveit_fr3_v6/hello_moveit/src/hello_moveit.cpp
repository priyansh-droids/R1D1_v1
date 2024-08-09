#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class HelloMoveit : public rclcpp::Node {
public:
    HelloMoveit()
    : Node("hello_moveit") {
        RCLCPP_INFO(this->get_logger(), "hello moveit has been initialized");
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/ChatGpt_arm_pose_data", 10,
            std::bind(&HelloMoveit::pose_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscription to /ChatGpt_arm_pose_data created successfully.");
    }

    void init_move_group() {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "fairino3_v6_group");
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface created successfully.");
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received target pose: [x: %f, y: %f, z: %f, w: %f]",
                    msg->position.x, msg->position.y, msg->position.z, msg->orientation.w);

        RCLCPP_INFO(this->get_logger(), "Setting pose target for MoveGroup...");
        move_group_->setPoseTarget(*msg);
        RCLCPP_INFO(this->get_logger(), "Pose target set.");

        RCLCPP_INFO(this->get_logger(), "Planning to target pose...");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(this->get_logger(), "Planning %s.", success ? "successful" : "failed");

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Executing the plan...");
            move_group_->execute(plan);
            RCLCPP_INFO(this->get_logger(), "Plan executed successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing HelloMoveit node...");
    auto node = std::make_shared<HelloMoveit>();
    node->init_move_group();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "HelloMoveit node initialized.");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Spinning node...");
    rclcpp::spin(node);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node spin completed.");

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down HelloMoveit node...");
    return 0;
}
