#include <rclcpp/rclcpp.hpp>
#include "imu_calib/do_calib.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create the calibration node
  auto calib_node = std::make_shared<imu_calib::DoCalib>();

  // Spin while the calibration node is running
  rclcpp::Rate rate(50);  // Optional loop rate
  while (rclcpp::ok() && calib_node->running())
  {
    rclcpp::spin_some(calib_node);
    rate.sleep();
  }

  RCLCPP_INFO(calib_node->get_logger(), "Calibration node shutting down.");
  rclcpp::shutdown();
  return 0;
}
