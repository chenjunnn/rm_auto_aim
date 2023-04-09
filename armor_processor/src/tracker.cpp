// Copyright 2022 Chen Jun

#include "armor_processor/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// STD
#include <cfloat>
#include <memory>
#include <string>

namespace rm_auto_aim
{
Tracker::Tracker(
  double max_match_distance, int tracking_threshold, int lost_threshold,
  Eigen::DiagonalMatrix<double, 9> q, Eigen::DiagonalMatrix<double, 4> r)
: tracker_state(LOST),
  tracked_id(std::string("")),
  target_state(Eigen::VectorXd::Zero(9)),
  max_match_distance_(max_match_distance),
  tracking_threshold_(tracking_threshold),
  lost_threshold_(lost_threshold)
{
  // Extended Kalman Filter
  // xa = x_armor, xc = x_robot_center
  // state: xc, yc, zc, yaw, v_xc, v_yc, v_zc, v_yaw, r
  // measurement: xa, ya, za, yaw
  // F - state transition matrix
  double dt = 0;  // Just for indication
  Eigen::Matrix<double, 9, 9> f;
  // clang-format off
  //    xc   yc   zc   yaw  vxc  vyc  vzc  vyaw r
  f <<  1,   0,   0,   0,   dt,  0,   0,   0,   0,
        0,   1,   0,   0,   0,   dt,  0,   0,   0,
        0,   0,   1,   0,   0,   0,   dt,  0,   0, 
        0,   0,   0,   1,   0,   0,   0,   dt,  0,
        0,   0,   0,   0,   1,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   1,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   1,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   1,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   1;

  // H - measurement matrix
  double yaw = 0;
  Eigen::Matrix<double, 4, 9> h;
  //    xc   yc   zc   yaw   vxc  vyc  vzc  vyaw r
  h <<  1,   0,   0,   0,    0,   0,   0,   0,   -cos(yaw),
        0,   1,   0,   0,    0,   0,   0,   0,   -sin(yaw),
        0,   0,   1,   0,    0,   0,   0,   0,   0,
        0,   0,   0,   1,    0,   0,   0,   0,   0;
  // clang-format on

  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 9> p;
  p.setIdentity();

  kf_matrices_ = KalmanFilterMatrices{f, h, q, r, p};
}

void Tracker::init(const Armors::SharedPtr & armors_msg)
{
  if (armors_msg->armors.empty()) {
    return;
  }

  // Simply choose the armor that is closest to image center
  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors[0];
  for (const auto & armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  initEKF(tracked_armor);

  tracked_id = tracked_armor.number;
  tracker_state = DETECTING;
}

void Tracker::update(const Armors::SharedPtr & armors_msg, const double & dt)
{
  // KF predict
  kf_->F(0, 4) = kf_->F(1, 5) = kf_->F(2, 6) = kf_->F(3, 7) = dt;
  Eigen::VectorXd kf_prediction = kf_->predict();
  RCLCPP_DEBUG(rclcpp::get_logger("armor_processor"), "KF predict");

  bool matched = false;
  // Use KF prediction as default target state if no matched armor is found
  target_state = kf_prediction;

  if (!armors_msg->armors.empty()) {
    double min_position_diff = DBL_MAX;
    for (const auto & armor : armors_msg->armors) {
      auto p = armor.pose.position;
      Eigen::Vector3d position_vec(p.x, p.y, p.z);
      // Difference of the current armor position and tracked armor's predicted position
      auto predicted_position = getArmorPositionFromRobotState(kf_prediction);
      double position_diff = (predicted_position - position_vec).norm();
      if (position_diff < min_position_diff) {
        min_position_diff = position_diff;
        tracked_armor = armor;
      }
    }

    if (min_position_diff < max_match_distance_) {
      // Matching armor found
      matched = true;
      auto p = tracked_armor.pose.position;
      // Update EKF
      double pre_yaw = kf_prediction(3);
      kf_->H(0, 8) = -cos(pre_yaw);
      kf_->H(1, 8) = -sin(pre_yaw);
      double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
      Eigen::Vector4d v(p.x, p.y, p.z, measured_yaw);
      target_state = kf_->update(v);
      RCLCPP_DEBUG(rclcpp::get_logger("armor_processor"), "KF update");
    } else {
      // Check if there is same id armor in current frame
      for (const auto & armor : armors_msg->armors) {
        if (armor.number == tracked_id) {
          // Armor jump happens
          matched = true;
          tracked_armor = armor;
          handleArmorJump(tracked_armor);
          break;
        }
      }
    }
  }

  // Suppress R from converging to zero
  if (target_state(8) < 0.2) {
    target_state(8) = 0.2;
    kf_->setState(target_state);
  }

  // Tracking state machine
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_threshold_) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_threshold_) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  }
}

void Tracker::initEKF(const Armor & a)
{
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.pose.orientation);

  // Set initial position at 0.2m behind the target
  target_state = Eigen::VectorXd::Zero(9);
  double r = 0.2;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  double zc = za;
  last_z = zc, last_r = r;
  target_state << xc, yc, zc, yaw, 0, 0, 0, 0, r;

  kf_ = std::make_unique<KalmanFilter>(kf_matrices_);
  kf_->setState(target_state);
}

void Tracker::handleArmorJump(const Armor & a)
{
  double last_yaw = target_state(3);
  double yaw = orientationToYaw(a.pose.orientation);

  if (abs(yaw - last_yaw) > 0.4) {
    last_z = target_state(2);
    target_state(2) = a.pose.position.z;
    target_state(3) = yaw;
    std::swap(target_state(8), last_r);
    RCLCPP_WARN(rclcpp::get_logger("armor_processor"), "Armor jump!");
  }

  auto p = a.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromRobotState(target_state);

  if ((current_p - infer_p).norm() > max_match_distance_) {
    double r = target_state(8);
    target_state(0) = p.x + r * cos(yaw);
    target_state(1) = p.y + r * sin(yaw);
    target_state(4) = 0;
    target_state(5) = 0;
    RCLCPP_ERROR(rclcpp::get_logger("armor_processor"), "Center wrong!");
  }

  kf_->setState(target_state);
}

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

Eigen::Vector3d Tracker::getArmorPositionFromRobotState(const Eigen::VectorXd & x)
{
  // Calculate predicted position of the current armor
  double xc = x(0), yc = x(1), zc = x(2);
  double yaw = x(3), r = x(8);
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, zc);
}

}  // namespace rm_auto_aim
