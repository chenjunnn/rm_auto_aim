// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>

#include "armor_processor/extended_kalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{
class Tracker
{
public:
  Tracker(double max_match_distance, int tracking_threshold, int lost_threshold);

  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  void init(const Armors::SharedPtr & armors_msg);

  void update(const Armors::SharedPtr & armors_msg);

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  ExtendedKalmanFilter ekf;

  Armor tracked_armor;
  std::string tracked_id;
  Eigen::VectorXd target_state;

  // To store another pair of armors message
  double last_z, last_r;

  // To store armor type
  uint8_t armor_type;

private:
  void initEKF(const Armor & a);

  void handleArmorJump(const Armor & a);

  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

  double max_match_distance_;

  int tracking_threshold_;
  int lost_threshold_;

  int detect_count_;
  int lost_count_;

  double last_yaw_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
