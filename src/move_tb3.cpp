/ Copyright(c) 2023 Manav Nagda manav19@umd.edu

/**
 * @file walker.cpp
 * @author Manav Nagda (manav19@umd.edu)
 * @brief Initiates the walker node and implements the walker algorithm
 * @version 0.1
 * @date 2023-11-28
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using LaserScanMsg = sensor_msgs::msg::LaserScan;
using TwistMsg = geometry_msgs::msg::Twist;

/**
 * @brief A class representing a simple walker node that responds to laser scan data.
 */
class RobotWalker : public rclcpp::Node {
public:
  /**
   * @brief Constructor for RobotWalker.
   */
  RobotWalker() : Node("move_tb3") {
    auto callback = std::bind(&RobotWalker::handleLaserData, this, _1);
    laserDataSubscription =
        this->create_subscription<LaserScanMsg>("scan", 10, callback);
    velocityPublisher = this->create_publisher<TwistMsg>("cmd_vel", 10);
  }

  /**
   * @brief Commands the robot to move based on specified velocities.
   * @param xVelocity Linear velocity along the x-axis.
   * @param zVelocity Angular velocity around the z-axis.
   */
  void moveRobot(float xVelocity, float zVelocity) {
    auto velocityMsg = TwistMsg();
    velocityMsg.linear.x = xVelocity;
    velocityMsg.angular.z = -zVelocity;
    velocityPublisher->publish(velocityMsg);
  }

private:
  /**
   * @brief Callback function to handle laser scan data.
   * @param scanData The received laser scan data.
   */
  void handleLaserData(const LaserScanMsg& scanData) {
    if (scanData.header.stamp.sec == 0) {
      return;
    }

    auto laserScanData = scanData.ranges;
    for (int i = 330; i < 330 + 60; i++) {
      if (laserScanData[i % 360] < 0.8) {
        moveRobot(0.0, 0.1);
      } else {
        moveRobot(0.1, 0.0);
      }
    }
  }

  // Private members
  rclcpp::Subscription<LaserScanMsg>::SharedPtr laserDataSubscription;
  rclcpp::Publisher<TwistMsg>::SharedPtr velocityPublisher;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotWalker>());
  rclcpp::shutdown();
  return 0;
}
