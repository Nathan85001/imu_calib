#include "imu_calib/mag_cal_node.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <fstream>
#include <string>

MagCalNode::MagCalNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mag_calibration_cpp", options)
{
  // ── Parameters ──────────────────────────────────────────────
  std::string calib_yaml = this->declare_parameter<std::string>("calib_yaml", "");
  declination_rad_       = this->declare_parameter<double>("mag_decl_deg", 4.216666667) * M_PI / 180.0;
  scale_factor_          = this->declare_parameter<double>("scale_factor", 1e-5);         // 0.1 µT → T
  int queue_depth        = this->declare_parameter<int>("queue_depth", 10);

  // ── Load YAML ───────────────────────────────────────────────
  YAML::Node root = YAML::LoadFile(calib_yaml);
  auto m = root["magnetometer"];

  auto hi_vec = m["hard_iron_offset"].as<std::vector<double>>();
  for (int i = 0; i < 3; ++i) hard_iron_(i) = hi_vec[i] * scale_factor_;

  auto si_rows = m["soft_iron_matrix"].as<std::vector<std::vector<double>>>();
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      soft_iron_(r, c) = si_rows[r][c];

  // ── Interfaces ──────────────────────────────────────────────
  mag_pub_     = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", queue_depth);
  heading_pub_ = this->create_publisher<std_msgs::msg::Float32>("/imu/heading", queue_depth);

  rclcpp::QoS qos(queue_depth);
  qos.best_effort();                // good for high-rate sensor; change to .reliable() if needed
  qos.durability_volatile();
  qos.keep_last(queue_depth);

  mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "/imu/raw_mag", qos,
      std::bind(&MagCalNode::magCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Mag calibration node started ✓ (yaml: %s)", calib_yaml.c_str());
}

void MagCalNode::magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
  // raw vector
  Eigen::Vector3d raw(msg->magnetic_field.x,
                      msg->magnetic_field.y,
                      msg->magnetic_field.z);

  // hard-iron compensation
  Eigen::Vector3d hi = raw - hard_iron_;

  // soft-iron compensation
  Eigen::Vector3d cal = soft_iron_ * hi;

  // heading (flat assumption, X east, Y north)
  double heading = std::fmod(
      -std::atan2(cal.x(), cal.y()) + declination_rad_ + 2.0 * M_PI,
      2.0 * M_PI);

  // ── publish calibrated vector ───────────────────────────────
  auto mag_msg = *msg;            // copy header + covariance if present
  mag_msg.magnetic_field.x = cal.x();
  mag_msg.magnetic_field.y = cal.y();
  mag_msg.magnetic_field.z = cal.z();
  mag_pub_->publish(mag_msg);

  // ── publish heading ─────────────────────────────────────────
  std_msgs::msg::Float32 h;
  h.data = static_cast<float>(heading * 180.0 / M_PI);   // degrees
  heading_pub_->publish(h);
}

// ── main ─────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MagCalNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
