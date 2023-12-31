/**
 * @file walker.cpp
 * @author Akashkumar Parmar (akasparm@umd.edu)
 * @brief
 * @version 0.1
 * @date 2023-11-29
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::chrono::duration_cast;
using std::chrono::seconds;
using std::placeholders::_1;

class WalkerBot : public rclcpp::Node {
 public:
  WalkerBot()
      : Node("bot_walker"), linear_velocity_(0.0), angular_velocity_(0.0) {
    // Initialize and retrieve parameters
    this->declare_parameter<float>("safe_distance", 0.8);
    this->get_parameter("safe_distance", safe_distance_);

    // Subscribe to laser scan data
    laser_scan_subscriber_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&WalkerBot::processLaserScan, this, _1));

    // Publisher for robot velocity
    velocity_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

 private:
  // Processes laser scan data and determines the movement of the robot
  void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    for (size_t i = 330; i < 390;
         ++i) {  // Check the front 60 degrees for obstacles
      if (scan_msg->ranges[i % 360] < safe_distance_) {
        linear_velocity_ = 0.0;
        angular_velocity_ = 0.1;
        publishVelocity();
        return;
      }
    }

    // No obstacle, move forward
    linear_velocity_ = 0.1;
    angular_velocity_ = 0.0;
    publishVelocity();
  }

  // Publishes velocity commands to the robot
  void publishVelocity() {
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = linear_velocity_;
    twist_msg.angular.z = -angular_velocity_;
    velocity_publisher_->publish(twist_msg);
  }

  // ROS 2 Subscribers and Publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  float linear_velocity_;
  float angular_velocity_;
  float safe_distance_;
};

int main(int argc, char** argv) {
  // Initialize and run the ROS 2 node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WalkerBot>());
  rclcpp::shutdown();
  return 0;
}
