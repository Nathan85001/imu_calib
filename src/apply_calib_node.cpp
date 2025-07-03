#include <rclcpp/rclcpp.hpp>
#include "imu_calib/apply_calib.hpp"  // Adjust path based on your package structure

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<imu_calib::ApplyCalib>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
