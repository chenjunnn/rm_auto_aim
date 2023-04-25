// Copyright 2022 Chen Jun
#include "armor_processor/processor_node.hpp"

// STD
#include <memory>
#include <vector>

namespace rm_auto_aim
{
ArmorProcessorNode::ArmorProcessorNode(const rclcpp::NodeOptions & options)
: Node("armor_processor", options), last_time_(0), dt_(0.0)
{
  RCLCPP_INFO(this->get_logger(), "Starting ProcessorNode!");

  // Tracker
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.2);
  int tracking_threshold = this->declare_parameter("tracker.tracking_threshold", 5);
  int lost_threshold = this->declare_parameter("tracker.lost_threshold", 5);
  tracker_ = std::make_unique<Tracker>(max_match_distance, tracking_threshold, lost_threshold);

  // EKF
  // xa = x_armor, xc = x_robot_center
  // state: xc, yc, zc, yaw, v_xc, v_yc, v_zc, v_yaw, r
  // measurement: xa, ya, za, yaw
  // f - Process function
  auto f = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(4) * dt_;
    x_new(1) += x(5) * dt_;
    x_new(2) += x(6) * dt_;
    x_new(3) += x(7) * dt_;
    return x_new;
  };
  // J_f - Jacobian of process function
  auto j_f = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(9, 9);
    // clang-format off
    f <<  1,   0,   0,   0,   dt_, 0,   0,   0,   0,
          0,   1,   0,   0,   0,   dt_, 0,   0,   0,
          0,   0,   1,   0,   0,   0,   dt_, 0,   0, 
          0,   0,   0,   1,   0,   0,   0,   dt_, 0,
          0,   0,   0,   0,   1,   0,   0,   0,   0,
          0,   0,   0,   0,   0,   1,   0,   0,   0,
          0,   0,   0,   0,   0,   0,   1,   0,   0,
          0,   0,   0,   0,   0,   0,   0,   1,   0,
          0,   0,   0,   0,   0,   0,   0,   0,   1;
    // clang-format on
    return f;
  };
  // h - Observation function
  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(1), yaw = x(3), r = x(8);
    z(0) = xc - r * cos(yaw);  // xa
    z(1) = yc - r * sin(yaw);  // ya
    z(2) = x(2);               // za
    z(3) = x(3);               // yaw
    return z;
  };
  // J_h - Jacobian of observation function
  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd h(4, 9);
    double yaw = x(3), r = x(8);
    // clang-format off
    //    xc   yc   zc   yaw         vxc  vyc  vzc  vyaw r
    h <<  1,   0,   0,   r*sin(yaw), 0,   0,   0,   0,   -cos(yaw),
          0,   1,   0,   -r*cos(yaw),0,   0,   0,   0,   -sin(yaw),
          0,   0,   1,   0,          0,   0,   0,   0,   0,
          0,   0,   0,   1,          0,   0,   0,   0,   0;
    // clang-format on
    return h;
  };
  // Q - process noise covariance matrix
  auto q_v = this->declare_parameter(
    "kf.q", std::vector<double>{//xc  yc    zc    yaw   vxc   vyc   vzc   vyaw  r
                                1e-2, 1e-2, 1e-2, 2e-2, 5e-2, 5e-2, 1e-4, 4e-2, 1e-3});
  Eigen::DiagonalMatrix<double, 9> q;
  q.diagonal() << q_v[0], q_v[1], q_v[2], q_v[3], q_v[4], q_v[5], q_v[6], q_v[7], q_v[8];
  // R - measurement noise covariance matrix
  auto r_v = this->declare_parameter(
    "kf.r", std::vector<double>{//xa  ya    za    yaw
                                1e-1, 1e-1, 1e-1, 2e-1});
  Eigen::DiagonalMatrix<double, 4> r;
  r.diagonal() << r_v[0], r_v[1], r_v[2], r_v[3];
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, q, r, p0};

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
  target_frame_ = this->declare_parameter("target_frame", "odom");
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
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armors_marker_.ns = "armors";
  armors_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  armors_marker_.scale.x = armors_marker_.scale.y = armors_marker_.scale.z = 0.1;
  armors_marker_.color.a = 1.0;
  armors_marker_.color.r = 1.0;
  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/processor/marker", 10);
}

void ArmorProcessorNode::armorsCallback(
  const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  // Tranform armor position from image frame to world coordinate
  for (auto & armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  auto_aim_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->update(armors_msg);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (tracker_->tracker_state == Tracker::TRACKING) {
      target_msg.tracking = true;
      target_msg.id = tracker_->tracked_id;
      tracker_->armor_type = armors_msg->armors[0].armor_type;
    } else if (tracker_->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;
      target_msg.id = tracker_->tracked_id;
    }
  }

  last_time_ = time;

  const auto state = tracker_->target_state;
  target_msg.position.x = state(0);
  target_msg.position.y = state(1);
  target_msg.position.z = state(2);
  target_msg.yaw = state(3);
  target_msg.velocity.x = state(4);
  target_msg.velocity.y = state(5);
  target_msg.velocity.z = state(6);
  target_msg.v_yaw = state(7);
  target_msg.radius_1 = state(8);
  target_msg.radius_2 = tracker_->last_r;
  target_msg.z_2 = tracker_->last_z;
  target_msg.armor_type = tracker_->armor_type;
  target_pub_->publish(target_msg);

  publishMarkers(target_msg);
}

void ArmorProcessorNode::publishMarkers(const auto_aim_interfaces::msg::Target & target_msg)
{
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armors_marker_.header = target_msg.header;

  if (target_msg.tracking) {
    auto state = tracker_->target_state;
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, zc = target_msg.position.z;
    double z2 = target_msg.z_2;
    uint8_t armor_type = target_msg.armor_type;
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = armor_type == 0 ? (zc + z2) / 2 : zc;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += state(4);
    arrow_end.y += state(5);
    arrow_end.z += state(6);
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += state(7) / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armors_marker_.action = visualization_msgs::msg::Marker::ADD;
    armors_marker_.points.clear();
    geometry_msgs::msg::Point p_a;
    if (armor_type == 0) {
      bool use_1 = true;
      for (size_t i = 0; i < 4; i++) {
        double tmp_yaw = yaw + i * M_PI_2;
        double r = use_1 ? r1 : r2;
        p_a.x = xc - r * cos(tmp_yaw);
        p_a.y = yc - r * sin(tmp_yaw);
        p_a.z = use_1 ? zc : z2;
        armors_marker_.points.emplace_back(p_a);
        use_1 = !use_1;
      }
    } else if (armor_type == 1) {
      for (size_t i = 0; i < 2; i++) {
        double tmp_yaw = yaw + i * M_PI;
        double r = r1;
        p_a.x = xc - r * cos(tmp_yaw);
        p_a.y = yc - r * sin(tmp_yaw);
        p_a.z = zc;
        armors_marker_.points.emplace_back(p_a);
      }
    }
  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    armors_marker_.action = visualization_msgs::msg::Marker::DELETE;
  }

  visualization_msgs::msg::MarkerArray marker_array;

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_array.markers.emplace_back(armors_marker_);
  marker_pub_->publish(marker_array);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorProcessorNode)
