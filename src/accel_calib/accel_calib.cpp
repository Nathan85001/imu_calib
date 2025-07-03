#include "imu_calib/accel_calib.hpp"  // Use .hpp since header uses that extension

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cstring> // for memset

namespace imu_calib
{

// Define the static arrays
const int AccelCalib::reference_index_[] = { 0, 0, 1, 1, 2, 2 };
const int AccelCalib::reference_sign_[] = { 1, -1, 1, -1, 1, -1 };

AccelCalib::AccelCalib()
: calib_ready_(false),
  calib_initialized_(false),
  num_measurements_(0),
  measurements_received_(0),
  reference_acceleration_(0.0)
{
  memset(orientation_count_, 0, sizeof(orientation_count_));
  SM_.setIdentity();
  bias_.setZero();
}

AccelCalib::AccelCalib(const std::string &calib_file)
: AccelCalib()  // Properly delegate to default constructor
{
  loadCalib(calib_file);
}

bool AccelCalib::calibReady() const
{
  return calib_ready_;
}

bool AccelCalib::loadCalib(const std::string &calib_file)
{
  try
  {
    YAML::Node node = YAML::LoadFile(calib_file);

    if (!node["SM"] || !node["bias"])
      return false;

    if (!node["SM"].IsSequence() || node["SM"].size() != 9)
      return false;

    if (!node["bias"].IsSequence() || node["bias"].size() != 3)
      return false;

    for (int i = 0; i < 9; i++)
    {
      SM_(i / 3, i % 3) = node["SM"][i].as<double>();
    }

    for (int i = 0; i < 3; i++)
    {
      bias_(i) = node["bias"][i].as<double>();
    }

    calib_ready_ = true;
    return true;
  }
  catch (...)
  {
    return false;
  }
}

bool AccelCalib::saveCalib(const std::string &calib_file) const
{
  if (!calib_ready_)
    return false;

  YAML::Node node;
  for (int i = 0; i < 9; i++)
  {
    node["accel_scale"].push_back(SM_(i / 3, i % 3));
  }

  for (int i = 0; i < 3; i++)
  {
    node["accel_bias"].push_back(bias_(i));
  }

  try
  {
    std::ofstream fout(calib_file);
    if (!fout.is_open())
      return false;

    fout << node;
    fout.close();
  }
  catch (...)
  {
    return false;
  }

  return true;
}

void AccelCalib::beginCalib(int measurements, double reference_acceleration)
{
  reference_acceleration_ = reference_acceleration;

  num_measurements_ = measurements;
  measurements_received_ = 0;

  meas_.resize(3 * measurements, 12);
  meas_.setZero();

  ref_.resize(3 * measurements);
  ref_.setZero();

  memset(orientation_count_, 0, sizeof(orientation_count_));
  calib_initialized_ = true;
  calib_ready_ = false; // reset ready flag since new calibration begins
}

bool AccelCalib::addMeasurement(Orientation orientation, double ax, double ay, double az)
{
  if (!calib_initialized_ || measurements_received_ >= num_measurements_)
    return false;

  // Fill measurement matrix
  for (int i = 0; i < 3; i++)
  {
    meas_(3 * measurements_received_ + i, 3 * i) = ax;
    meas_(3 * measurements_received_ + i, 3 * i + 1) = ay;
    meas_(3 * measurements_received_ + i, 3 * i + 2) = az;
    meas_(3 * measurements_received_ + i, 9 + i) = -1.0;
  }

  // Fill reference vector
  ref_(3 * measurements_received_ + reference_index_[orientation]) =
      reference_sign_[orientation] * reference_acceleration_;

  measurements_received_++;
  orientation_count_[orientation]++;

  return true;
}

bool AccelCalib::computeCalib()
{
  if (measurements_received_ < 12)
    return false;

  for (int i = 0; i < 6; i++)
  {
    if (orientation_count_[i] == 0)
      return false;
  }

  // Solve least squares problem
  Eigen::VectorXd xhat = meas_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(ref_);

  // Extract calibration parameters
  for (int i = 0; i < 9; i++)
  {
    SM_(i / 3, i % 3) = xhat(i);
  }

  for (int i = 0; i < 3; i++)
  {
    bias_(i) = xhat(9 + i);
  }

  calib_ready_ = true;
  return true;
}

void AccelCalib::applyCalib(const double raw[3], double corrected[3]) const
{
  Eigen::Vector3d raw_accel(raw[0], raw[1], raw[2]);
  Eigen::Vector3d corrected_accel = SM_ * raw_accel - bias_;

  corrected[0] = corrected_accel(0);
  corrected[1] = corrected_accel(1);
  corrected[2] = corrected_accel(2);
}

void AccelCalib::applyCalib(double raw_x, double raw_y, double raw_z,
                            double *corr_x, double *corr_y, double *corr_z) const
{
  Eigen::Vector3d raw_accel(raw_x, raw_y, raw_z);
  Eigen::Vector3d corrected_accel = SM_ * raw_accel - bias_;

  *corr_x = corrected_accel(0);
  *corr_y = corrected_accel(1);
  *corr_z = corrected_accel(2);
}

}  // namespace imu_calib
