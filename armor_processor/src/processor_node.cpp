// Copyright 2022 Chen Jun

#include "armor_processor/processor_node.hpp"

// STD
#include <memory>
#include <vector>

#include "armor_processor/kalman_filter.hpp"

namespace rm_auto_aim
{
ArmorProcessorNode::ArmorProcessorNode(const rclcpp::NodeOptions & options)
: Node("armor_processor", options), last_time_(0), dt_(0.0)
{
  RCLCPP_INFO(this->get_logger(), "Starting ProcessorNode!");

  // Kalman Filter initial matrix
  // A - state transition matrix
  // clang-format off
  Eigen::Matrix<double, 6, 6> a;
  a <<  1,  0,  0, dt_, 0,  0,
        0,  1,  0,  0, dt_, 0,
        0,  0,  1,  0,  0, dt_,
        0,  0,  0,  1,  0,  0,
        0,  0,  0,  0,  1,  0,
        0,  0,  0,  0,  0,  1;
  // clang-format on

  // H - measurement matrix
  Eigen::Matrix<double, 3, 6> h;
  h.setIdentity();

  // Q - process noise covariance matrix
  Eigen::DiagonalMatrix<double, 6> q;
  q.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1;

  // R - measurement noise covariance matrix
  Eigen::DiagonalMatrix<double, 3> r;
  r.diagonal() << 0.05, 0.05, 0.05;

  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 6> p;
  p.setIdentity();

  kf_matrices_ = KalmanFilterMatrices{a, h, q, r, p};

  // Tracker
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.2);
  int tracking_threshold = this->declare_parameter("tracker.tracking_threshold", 5);
  int lost_threshold = this->declare_parameter("tracker.lost_threshold", 5);
  tracker_ = std::make_unique<Tracker>(max_match_distance, tracking_threshold, lost_threshold);

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "shooter_link");
  tf2_filter_ = std::make_shared<tf2_filter>(
    armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&ArmorProcessorNode::armorsCallback, this);

  // Publisher
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/processor/target", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.header.frame_id = target_frame_;
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  velocity_marker_.header.frame_id = target_frame_;
  velocity_marker_.type = visualization_msgs::msg::Marker::ARROW;
  position_marker_.ns = "velocity";
  velocity_marker_.scale.x = 0.03;
  velocity_marker_.scale.y = 0.05;
  velocity_marker_.color.a = 1.0;
  velocity_marker_.color.b = 1.0;
  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/processor/marker", 10);

  // Debug Publishers
  debug_ = this->declare_parameter("debug", true);
  // if (debug_) {
  // }

  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
      debug_ = p.as_bool();
      // debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });
}

void ArmorProcessorNode::armorsCallback(
  const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  // Tranform armor position from image frame to world coordinate
  for (auto & armor : armors_msg->armors) {
    geometry_msgs::msg::PointStamped ps;
    ps.header = armors_msg->header;
    ps.point = armor.position;
    try {
      armor.position = tf2_buffer_->transform(ps, target_frame_).point;
    } catch (const tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  rclcpp::Time time = armors_msg->header.stamp;
  target_msg_.header.stamp = time;

  if (tracker_->tracker_state == Tracker::NO_FOUND) {
    tracker_->init(armors_msg, kf_matrices_);

    target_msg_.target_found = false;
    target_pub_->publish(target_msg_);
    deleteMarkers();

  } else {
    // Set dt
    dt_ = (time - last_time_).seconds();
    kf_matrices_.A(0, 3) = kf_matrices_.A(1, 4) = kf_matrices_.A(2, 5) = dt_;
    tracker_->update(armors_msg, kf_matrices_.A);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg_.target_found = false;
      target_pub_->publish(target_msg_);
      deleteMarkers();

    } else if (
      tracker_->tracker_state == Tracker::TRACKING || tracker_->tracker_state == Tracker::LOST) {
      target_msg_.target_found = true;
      publishTarget(tracker_->target_state);
      publishMarkers(time, tracker_->target_state);
    }
  }

  last_time_ = time;

  if (debug_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Tracker state:" << tracker_->tracker_state);
  }
}

void ArmorProcessorNode::deleteMarkers()
{
  position_marker_.action = visualization_msgs::msg::Marker::DELETE;
  velocity_marker_.action = visualization_msgs::msg::Marker::DELETE;
  marker_array_.markers.clear();
  marker_array_.markers.emplace_back(position_marker_);
  marker_array_.markers.emplace_back(velocity_marker_);
  marker_pub_->publish(marker_array_);
}

void ArmorProcessorNode::publishMarkers(const rclcpp::Time & time, const Eigen::VectorXd & state)
{
  position_marker_.action = visualization_msgs::msg::Marker::ADD;
  position_marker_.header.stamp = time;
  position_marker_.pose.position.x = state(0);
  position_marker_.pose.position.y = state(1);
  position_marker_.pose.position.z = state(2);

  velocity_marker_.action = visualization_msgs::msg::Marker::ADD;
  velocity_marker_.header.stamp = time;
  velocity_marker_.points.clear();
  geometry_msgs::msg::Point p;
  p.x = state(0);
  p.y = state(1);
  p.z = state(2);
  velocity_marker_.points.emplace_back(p);
  p.x += state(3);
  p.y += state(4);
  p.z += state(5);
  velocity_marker_.points.emplace_back(p);

  marker_array_.markers.clear();
  marker_array_.markers.emplace_back(position_marker_);
  marker_array_.markers.emplace_back(velocity_marker_);
  marker_pub_->publish(marker_array_);
}

void ArmorProcessorNode::publishTarget(const Eigen::VectorXd & kf_state)
{
  target_msg_.position.x = kf_state(0);
  target_msg_.position.y = kf_state(1);
  target_msg_.position.z = kf_state(2);
  target_msg_.velocity.x = kf_state(3);
  target_msg_.velocity.y = kf_state(4);
  target_msg_.velocity.z = kf_state(5);
  target_pub_->publish(target_msg_);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorProcessorNode)
