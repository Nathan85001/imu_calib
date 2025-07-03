#include "imu_calib/do_calib.hpp"

#include <iostream>

namespace imu_calib
{

DoCalib::DoCalib() : Node("do_calib"), state_(START), measurements_received_(0)
{
  // Declare parameters with default values
  this->declare_parameter<int>("measurements", 500);
  this->declare_parameter<double>("reference_acceleration", 9.80665);
  this->declare_parameter<std::string>("output_file", "imu_calib.yaml");

  // Get parameter values
  this->get_parameter("measurements", measurements_per_orientation_);
  this->get_parameter("reference_acceleration", reference_acceleration_);
  this->get_parameter("output_file", output_file_);

  // Initialize orientations queue
  orientations_.push(AccelCalib::XPOS);
  orientations_.push(AccelCalib::XNEG);
  orientations_.push(AccelCalib::YPOS);
  orientations_.push(AccelCalib::YNEG);
  orientations_.push(AccelCalib::ZPOS);
  orientations_.push(AccelCalib::ZNEG);

  // Orientation labels
  orientation_labels_[AccelCalib::XPOS] = "X+";
  orientation_labels_[AccelCalib::XNEG] = "X-";
  orientation_labels_[AccelCalib::YPOS] = "Y+";
  orientation_labels_[AccelCalib::YNEG] = "Y-";
  orientation_labels_[AccelCalib::ZPOS] = "Z+";
  orientation_labels_[AccelCalib::ZNEG] = "Z-";

  // Subscribe to IMU topic
  rclcpp::SensorDataQoS qos;
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", qos,
    std::bind(&DoCalib::imuCallback, this, std::placeholders::_1));
}

bool DoCalib::running()
{
  return state_ != DONE;
}

void DoCalib::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu)
{
  bool accepted;

  switch (state_)
  {
    case START:
      calib_.beginCalib(6 * measurements_per_orientation_, reference_acceleration_);
      state_ = SWITCHING;
      break;

    case SWITCHING:
      if (orientations_.empty())
      {
        state_ = COMPUTING;
      }
      else
      {
        current_orientation_ = orientations_.front();
        orientations_.pop();
        measurements_received_ = 0;

        std::cout << "\nOrient IMU with " << orientation_labels_[current_orientation_] 
                  << " axis up and press Enter...";
        std::cin.get();
        std::cout << "Recording measurements..." << std::endl;

        state_ = RECEIVING;
      }
      break;

    case RECEIVING:
      accepted = calib_.addMeasurement(current_orientation_,
                                       imu->linear_acceleration.x,
                                       imu->linear_acceleration.y,
                                       imu->linear_acceleration.z);

      if (accepted)
        ++measurements_received_;

      if (measurements_received_ >= measurements_per_orientation_)
      {
        std::cout << " Done." << std::endl;
        state_ = SWITCHING;
      }
      break;

    case COMPUTING:
      std::cout << "Computing calibration parameters..." << std::endl;
      if (calib_.computeCalib())
      {
        std::cout << "Success!" << std::endl;

        std::cout << "Saving calibration file...";
        if (calib_.saveCalib(output_file_))
        {
          std::cout << " Success!" << std::endl;
        }
        else
        {
          std::cout << " Failed." << std::endl;
          RCLCPP_ERROR(this->get_logger(), "Failed to save calibration file");
        }
      }
      else
      {
        std::cout << " Failed." << std::endl;
        RCLCPP_ERROR(this->get_logger(), "Calibration computation failed");
      }
      state_ = DONE;
      break;

    case DONE:
      // Do nothing; calibration finished
      break;
  }
}

}  // namespace imu_calib
