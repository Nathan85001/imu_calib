/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Daniel Koch
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef IMU_CALIB__APPLY_CALIB_HPP_
#define IMU_CALIB__APPLY_CALIB_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include <string>
#include <vector>
#include <fstream>

namespace imu_calib
{

class ApplyCalib : public rclcpp::Node
{
public:
  ApplyCalib();
  ~ApplyCalib();

private:
  // ROS 2 communication
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr corrected_imu_pub_;

  // Parameters
  std::string calib_file_;
  bool calibrate_gyros_;
  int gyro_calib_samples_;

  // Calibration data
  int gyro_sample_count_;
  Eigen::Vector3d gyro_sum_;
  Eigen::Vector3d gyro_bias_;
  Eigen::Matrix3d accel_scale_;
  Eigen::Vector3d accel_bias_;  
  bool gyro_calibrated_;



  // Callbacks and utilities
  void rawImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  bool loadCalibration();
  void calibrateGyros(const sensor_msgs::msg::Imu::SharedPtr msg);
  sensor_msgs::msg::Imu applyCorrectionAccel(const sensor_msgs::msg::Imu::SharedPtr msg) const;
  sensor_msgs::msg::Imu applyCorrectionGyro(const sensor_msgs::msg::Imu& msg) const;
};

}  // namespace imu_calib

#endif  // IMU_CALIB__APPLY_CALIB_HPP_
