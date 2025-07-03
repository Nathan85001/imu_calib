#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float32.hpp>
#include <Eigen/Dense>

class MagCalNode : public rclcpp::Node
{
public:
  explicit MagCalNode(const rclcpp::NodeOptions & options);

private:
  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg);

  Eigen::Vector3d hard_iron_;
  Eigen::Matrix3d soft_iron_;
  double declination_rad_;
  double scale_factor_;

  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
};
