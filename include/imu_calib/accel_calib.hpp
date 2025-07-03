#ifndef IMU_CALIB__ACCEL_CALIB_HPP_
#define IMU_CALIB__ACCEL_CALIB_HPP_

#include <Eigen/Dense>
#include <string>

namespace imu_calib
{

class AccelCalib
{
public:
  enum Orientation { XPOS = 0, XNEG, YPOS, YNEG, ZPOS, ZNEG };

  AccelCalib();
  explicit AccelCalib(const std::string &calib_file);

  // Status
  bool calibReady() const;

  // File I/O
  bool loadCalib(const std::string &calib_file);
  bool saveCalib(const std::string &calib_file) const;

  // Calibration procedure
  void beginCalib(int measurements, double reference_acceleration);
  bool addMeasurement(Orientation orientation, double ax, double ay, double az);
  bool computeCalib();

  // Apply calibration
  void applyCalib(const double raw[3], double corrected[3]) const;
  void applyCalib(double raw_x, double raw_y, double raw_z,
                  double *corr_x, double *corr_y, double *corr_z) const;

protected:
  static const int reference_index_[6];
  static const int reference_sign_[6];

  bool calib_ready_;
  bool calib_initialized_;

  int orientation_count_[6];
  int num_measurements_;
  int measurements_received_;

  Eigen::Matrix3d SM_;          // Scale and misalignment matrix
  Eigen::Vector3d bias_;        // Bias vector

  double reference_acceleration_;

  Eigen::MatrixXd meas_; // Least squares matrix of measurements
  Eigen::VectorXd ref_;  // Least squares vector of expected values
};

}  // namespace imu_calib

#endif  // IMU_CALIB__ACCEL_CALIB_HPP_
