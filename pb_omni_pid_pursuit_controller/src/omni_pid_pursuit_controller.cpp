// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace pb_omni_pid_pursuit_controller
{

static constexpr double kPi = 3.14159265358979323846;

static inline double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

static inline double deg2rad(double deg)
{
  return deg * kPi / 180.0;
}

static inline double rad2deg(double rad)
{
  return rad * 180.0 / kPi;
}

void OmniPidPursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 1.0;
  double control_frequency = 20.0;
  max_robot_pose_search_dist_ = getCostmapMaxExtent();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kd", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_rotation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kd", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_max_sum_error", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_interpolation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading_treshold", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_min", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_min", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(getCostmapMaxExtent()));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_min", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_max", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".reduction_ratio_at_high_curvature", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_forward_dist", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_backward_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_velocity_scaling_factor_rate", rclcpp::ParameterValue(0.9));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".align_to_target_before_motion", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".align_to_target_use_rear_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".align_to_target_heading_frame", rclcpp::ParameterValue(std::string("")));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".align_to_target_threshold_deg", rclcpp::ParameterValue(30.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".align_to_target_tolerance_deg", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".align_to_target_rotation_kp", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".align_to_target_disable_distance", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".align_to_target_debug", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".align_to_target_debug_throttle_sec", rclcpp::ParameterValue(1.0));

  node->get_parameter(plugin_name_ + ".translation_kp", translation_kp_);
  node->get_parameter(plugin_name_ + ".translation_ki", translation_ki_);
  node->get_parameter(plugin_name_ + ".translation_kd", translation_kd_);
  node->get_parameter(plugin_name_ + ".enable_rotation", enable_rotation_);
  node->get_parameter(plugin_name_ + ".rotation_kp", rotation_kp_);
  node->get_parameter(plugin_name_ + ".rotation_ki", rotation_ki_);
  node->get_parameter(plugin_name_ + ".rotation_kd", rotation_kd_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".min_max_sum_error", min_max_sum_error_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".use_interpolation", use_interpolation_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(
    plugin_name_ + ".use_rotate_to_heading_treshold", use_rotate_to_heading_treshold_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  if (approach_velocity_scaling_dist_ > costmap_->getSizeInMetersX() / 2.0) {
    RCLCPP_WARN(
      logger_,
      "approach_velocity_scaling_dist is larger than forward costmap extent, "
      "leading to permanent slowdown");
  }
  node->get_parameter(plugin_name_ + ".v_linear_max", v_linear_max_);
  node->get_parameter(plugin_name_ + ".v_linear_min", v_linear_min_);
  node->get_parameter(plugin_name_ + ".v_angular_max", v_angular_max_);
  node->get_parameter(plugin_name_ + ".v_angular_min", v_angular_min_);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
  node->get_parameter(plugin_name_ + ".curvature_min", curvature_min_);
  node->get_parameter(plugin_name_ + ".curvature_max", curvature_max_);
  node->get_parameter(
    plugin_name_ + ".reduction_ratio_at_high_curvature", reduction_ratio_at_high_curvature_);
  node->get_parameter(plugin_name_ + ".curvature_forward_dist", curvature_forward_dist_);
  node->get_parameter(plugin_name_ + ".curvature_backward_dist", curvature_backward_dist_);
  node->get_parameter(
    plugin_name_ + ".max_velocity_scaling_factor_rate", max_velocity_scaling_factor_rate_);

  node->get_parameter(
    plugin_name_ + ".align_to_target_before_motion", align_to_target_before_motion_);
  node->get_parameter(
    plugin_name_ + ".align_to_target_use_rear_heading", align_to_target_use_rear_heading_);
  node->get_parameter(
    plugin_name_ + ".align_to_target_heading_frame", align_to_target_heading_frame_);
  node->get_parameter(
    plugin_name_ + ".align_to_target_threshold_deg", align_to_target_threshold_deg_);
  node->get_parameter(
    plugin_name_ + ".align_to_target_tolerance_deg", align_to_target_tolerance_deg_);
  node->get_parameter(
    plugin_name_ + ".align_to_target_rotation_kp", align_to_target_rotation_kp_);
  node->get_parameter(
    plugin_name_ + ".align_to_target_disable_distance", align_to_target_disable_distance_);
  node->get_parameter(
    plugin_name_ + ".align_to_target_debug", align_to_target_debug_);
  node->get_parameter(
    plugin_name_ + ".align_to_target_debug_throttle_sec", align_to_target_debug_throttle_sec_);

  node->get_parameter("controller_frequency", control_frequency);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;

  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  curvature_points_pub_ =
    node_.lock()
      ->create_publisher<visualization_msgs::msg::MarkerArray>(  // 初始化 MarkerArray Publisher
        "curvature_points_marker_array", rclcpp::QoS(10));

  move_pid_ = std::make_shared<PID>(
    control_duration_, v_linear_max_, v_linear_min_, translation_kp_, translation_kd_,
    translation_ki_);
  heading_pid_ = std::make_shared<PID>(
    control_duration_, v_angular_max_, v_angular_min_, rotation_kp_, rotation_kd_, rotation_ki_);

  align_to_target_active_ = false;
}

void OmniPidPursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " pb_omni_pid_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_.reset();
  carrot_pub_.reset();
  curvature_points_pub_.reset();
}

void OmniPidPursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "regulated_pure_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_->on_activate();
  carrot_pub_->on_activate();
  curvature_points_pub_->on_activate();
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&OmniPidPursuitController::dynamicParametersCallback, this, std::placeholders::_1));
}

void OmniPidPursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "regulated_pure_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  curvature_points_pub_->on_deactivate();
  dyn_params_handler_.reset();
}

geometry_msgs::msg::TwistStamped OmniPidPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(velocity);

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double lin_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  double theta_dist = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  double angle_to_goal = tf2::getYaw(carrot_pose.pose.orientation);

  if (use_rotate_to_heading_) {
    angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    if (fabs(angle_to_goal) > use_rotate_to_heading_treshold_) {
      lin_dist = 0;
    }
  }

  auto lin_vel = move_pid_->calculate(lin_dist, 0);
  auto angular_vel = enable_rotation_ ? heading_pid_->calculate(angle_to_goal, 0) : 0.0;

  applyCurvatureLimitation(transformed_plan, carrot_pose, lin_vel);

  applyApproachVelocityScaling(transformed_plan, lin_vel);

  // If the angle between (rear/front) heading and the target point is too large,
  // rotate in place until aligned, then continue translating.
  if (align_to_target_before_motion_) {
    // Disable this behavior near the final goal pose.
    if (!transformed_plan.poses.empty() && align_to_target_disable_distance_ > 0.0) {
      const auto & goal_local = transformed_plan.poses.back().pose.position;
      const double dist_to_goal = std::hypot(goal_local.x, goal_local.y);
      if (dist_to_goal <= align_to_target_disable_distance_) {
        align_to_target_active_ = false;
        goto skip_align_to_target;
      }
    }

    double align_error = 0.0;
    bool has_align_error = false;
    bool used_tf_global_yaw = false;
    double robot_yaw = std::numeric_limits<double>::quiet_NaN();
    double bearing = std::numeric_limits<double>::quiet_NaN();

    // Preferred: compute using map/global yaw from a specific TF frame (e.g., chassis)
    // This matches the idea in robot_pose_monitor.py (map -> chassis yaw).
    if (!align_to_target_heading_frame_.empty()) {
      const auto global_frame =
        !global_plan_.header.frame_id.empty() ? global_plan_.header.frame_id : pose.header.frame_id;
      try {
        const geometry_msgs::msg::TransformStamped tf_global_to_heading =
          tf_->lookupTransform(global_frame, align_to_target_heading_frame_, tf2::TimePointZero);

        const double robot_x = tf_global_to_heading.transform.translation.x;
        const double robot_y = tf_global_to_heading.transform.translation.y;
        robot_yaw = tf2::getYaw(tf_global_to_heading.transform.rotation);

        geometry_msgs::msg::PoseStamped carrot_global;
        if (transformPose(global_frame, carrot_pose, carrot_global)) {
          const double target_x = carrot_global.pose.position.x;
          const double target_y = carrot_global.pose.position.y;

          bearing = std::atan2(target_y - robot_y, target_x - robot_x);
          const double heading = robot_yaw + (align_to_target_use_rear_heading_ ? kPi : 0.0);
          align_error = normalize_angle(bearing - heading);
          has_align_error = true;
          used_tf_global_yaw = true;
        }
      } catch (tf2::TransformException & /*ex*/) {
        has_align_error = false;
      }
    }

    // If TF lookup failed, try to get yaw from the controller pose transformed into the global frame.
    if (!used_tf_global_yaw) {
      const auto global_frame =
        !global_plan_.header.frame_id.empty() ? global_plan_.header.frame_id : pose.header.frame_id;
      geometry_msgs::msg::PoseStamped pose_global;
      if (transformPose(global_frame, pose, pose_global)) {
        robot_yaw = tf2::getYaw(pose_global.pose.orientation);
      }
    }

    // Fallback: use carrot point in robot base frame (theta_dist).
    if (!has_align_error) {
      align_error = align_to_target_use_rear_heading_ ? normalize_angle(theta_dist - kPi) : theta_dist;
      has_align_error = true;
    }

    if (align_to_target_debug_) {
      const int64_t throttle_ms =
        static_cast<int64_t>(std::max(0.05, align_to_target_debug_throttle_sec_) * 1000.0);

      // Only print the 2 requested values: map/global yaw and angle to target.
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, throttle_ms,
        "[align_to_target] yaw=%.1fdeg angle=%.1fdeg",
        rad2deg(robot_yaw), rad2deg(align_error));
    }

    const double threshold = deg2rad(std::max(0.0, align_to_target_threshold_deg_));
    const double tolerance = deg2rad(std::max(0.0, align_to_target_tolerance_deg_));

    if (!align_to_target_active_) {
      if (std::fabs(align_error) > threshold) {
        align_to_target_active_ = true;
        heading_pid_->setSumError(0.0);
      }
    } else {
      if (std::fabs(align_error) <= tolerance) {
        align_to_target_active_ = false;
        heading_pid_->setSumError(0.0);
      }
    }

    if (align_to_target_active_) {
      geometry_msgs::msg::TwistStamped cmd_vel;
      cmd_vel.header = pose.header;
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.linear.y = 0.0;
      cmd_vel.twist.angular.z = std::clamp(
        align_to_target_rotation_kp_ * align_error, v_angular_min_, v_angular_max_);
      return cmd_vel;
    }
  }

skip_align_to_target:

  // Transform local frame to global frame to use in collision checking
  nav_msgs::msg::Path costmap_frame_local_plan;

  int sample_points = 10;
  int plan_size = transformed_plan.poses.size();
  for (int i = 0; i < sample_points; ++i) {
    int index = std::min((i * plan_size) / sample_points, plan_size - 1);
    geometry_msgs::msg::PoseStamped map_pose;
    transformPose(costmap_ros_->getGlobalFrameID(), transformed_plan.poses[index], map_pose);
    costmap_frame_local_plan.poses.push_back(map_pose);
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  if (!isCollisionDetected(costmap_frame_local_plan)) {
    cmd_vel.twist.linear.x = lin_vel * cos(theta_dist);
    cmd_vel.twist.linear.y = lin_vel * sin(theta_dist);
    cmd_vel.twist.angular.z = angular_vel;
  } else {
    throw nav2_core::PlannerException("Collision detected in the trajectory. Stopping the robot!");
  }

  return cmd_vel;
}

void OmniPidPursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  align_to_target_active_ = false;
}

void OmniPidPursuitController::setSpeedLimit(
  const double & /*speed_limit*/, const bool & /*percentage*/)
{
  RCLCPP_WARN(logger_, "Speed limit is not implemented in this controller.");
}

nav_msgs::msg::Path OmniPidPursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  double max_costmap_extent = getCostmapMaxExtent();

  auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin = nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) { return euclidean_distance(pose, robot_pose) > max_costmap_extent; });

  // Lambda to transform a PoseStamped from global frame to local
  auto transform_global_pose_to_local = [&](const auto & global_plan_pose) {
    geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
    stamped_pose.header.frame_id = global_plan_.header.frame_id;
    stamped_pose.header.stamp = robot_pose.header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
    transformed_pose.pose.position.z = 0.0;
    return transformed_pose;
  };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
    transform_global_pose_to_local);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  local_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

std::unique_ptr<geometry_msgs::msg::PointStamped> OmniPidPursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::getLookAheadPoint(
  const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    return pose;
  }

  return *goal_pose_it;
}

geometry_msgs::msg::Point OmniPidPursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double d = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - d * d);
  p.x = (d * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-d * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

double OmniPidPursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters =
    std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}
bool OmniPidPursuitController::transformPose(
  const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

bool OmniPidPursuitController::isCollisionDetected(const nav_msgs::msg::Path & path)
{
  auto costmap = costmap_ros_->getCostmap();
  for (const auto & pose_stamped : path.poses) {
    const auto & pose = pose_stamped.pose;
    unsigned int mx, my;
    if (costmap->worldToMap(pose.position.x, pose.position.y, mx, my)) {
      if (costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return true;
      }
    } else {
      // RCLCPP_WARN(
      //   logger_,
      //   "The Local path is not in the costmap. Cannot check for collisions. "
      //   "Proceed at your own risk, slow the robot, or increase your costmap size.");
      return false;
    }
  }
  return false;
}

double OmniPidPursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;

  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = hypot(speed.linear.x, speed.linear.y) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}

double OmniPidPursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path) const
{
  // Waiting to apply the threshold based on integrated distance ensures we don't
  // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
  double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = transformed_path.poses.back();
    // Here we will use a regular euclidean distance from the robot frame (origin)
    // to get smooth scaling, regardless of path density.
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

void OmniPidPursuitController::applyApproachVelocityScaling(
  const nav_msgs::msg::Path & path, double & linear_vel) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

  // Use the lowest velocity between approach and other constraints, if all overlapping
  linear_vel = std::min(linear_vel, approach_vel);
}

void OmniPidPursuitController::applyCurvatureLimitation(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double & linear_vel)
{
  double curvature =
    calculateCurvature(path, lookahead_pose, curvature_forward_dist_, curvature_backward_dist_);

  double scaled_linear_vel = linear_vel;
  if (curvature > curvature_min_) {
    double reduction_ratio = 1.0;
    if (curvature > curvature_max_) {
      reduction_ratio = reduction_ratio_at_high_curvature_;
    } else {
      reduction_ratio = 1.0 - (curvature - curvature_min_) / (curvature_max_ - curvature_min_) *
                                (1.0 - reduction_ratio_at_high_curvature_);
    }

    double target_scaled_vel = linear_vel * reduction_ratio;
    scaled_linear_vel =
      last_velocity_scaling_factor_ + std::clamp(
                                        target_scaled_vel - last_velocity_scaling_factor_,
                                        -max_velocity_scaling_factor_rate_ * control_duration_,
                                        max_velocity_scaling_factor_rate_ * control_duration_);
  }
  scaled_linear_vel = std::max(scaled_linear_vel, 2.0 * min_approach_linear_velocity_);

  linear_vel = std::min(linear_vel, scaled_linear_vel);
  last_velocity_scaling_factor_ = linear_vel;
}

double OmniPidPursuitController::calculateCurvature(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double forward_dist, double backward_dist) const
{
  geometry_msgs::msg::PoseStamped backward_pose, forward_pose;
  std::vector<double> cumulative_distances = calculateCumulativeDistances(path);

  double lookahead_pose_cumulative_distance = 0.0;
  geometry_msgs::msg::PoseStamped robot_base_frame_pose;
  robot_base_frame_pose.pose = geometry_msgs::msg::Pose();
  lookahead_pose_cumulative_distance =
    nav2_util::geometry_utils::euclidean_distance(robot_base_frame_pose, lookahead_pose);

  backward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance - backward_dist);

  forward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance + forward_dist);

  double curvature_radius = calculateCurvatureRadius(
    backward_pose.pose.position, lookahead_pose.pose.position, forward_pose.pose.position);
  double curvature = 1.0 / curvature_radius;
  visualizeCurvaturePoints(backward_pose, forward_pose);
  return curvature;
}

double OmniPidPursuitController::calculateCurvatureRadius(
  const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
  const geometry_msgs::msg::Point & far_point) const
{
  double x1 = near_point.x, y1 = near_point.y;
  double x2 = current_point.x, y2 = current_point.y;
  double x3 = far_point.x, y3 = far_point.y;

  double center_x = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
                     (x3 * x3 + y3 * y3) * (y1 - y2)) /
                    (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
  double center_y = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
                     (x3 * x3 + y3 * y3) * (x2 - x1)) /
                    (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
  double radius = std::hypot(x2 - center_x, y2 - center_y);
  if (std::isnan(radius) || std::isinf(radius) || radius < 1e-9) {
    return 1e9;
  }
  return radius;
}

void OmniPidPursuitController::visualizeCurvaturePoints(
  const geometry_msgs::msg::PoseStamped & backward_pose,
  const geometry_msgs::msg::PoseStamped & forward_pose) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker near_marker;
  near_marker.header = backward_pose.header;
  near_marker.ns = "curvature_points";
  near_marker.id = 0;
  near_marker.type = visualization_msgs::msg::Marker::SPHERE;
  near_marker.action = visualization_msgs::msg::Marker::ADD;
  near_marker.pose = backward_pose.pose;
  near_marker.scale.x = near_marker.scale.y = near_marker.scale.z = 0.1;
  near_marker.color.g = 1.0;
  near_marker.color.a = 1.0;

  visualization_msgs::msg::Marker far_marker;
  far_marker.header = forward_pose.header;
  far_marker.ns = "curvature_points";
  far_marker.id = 1;
  far_marker.type = visualization_msgs::msg::Marker::SPHERE;
  far_marker.action = visualization_msgs::msg::Marker::ADD;
  far_marker.pose = forward_pose.pose;
  far_marker.scale.x = far_marker.scale.y = far_marker.scale.z = 0.1;
  far_marker.color.r = 1.0;
  far_marker.color.a = 1.0;

  marker_array.markers.push_back(near_marker);
  marker_array.markers.push_back(far_marker);

  curvature_points_pub_->publish(marker_array);
}

std::vector<double> OmniPidPursuitController::calculateCumulativeDistances(
  const nav_msgs::msg::Path & path) const
{
  std::vector<double> cumulative_distances;
  cumulative_distances.push_back(0.0);

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & prev_pose = path.poses[i - 1].pose.position;
    const auto & curr_pose = path.poses[i].pose.position;
    double distance = hypot(curr_pose.x - prev_pose.x, curr_pose.y - prev_pose.y);
    cumulative_distances.push_back(cumulative_distances.back() + distance);
  }
  return cumulative_distances;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::findPoseAtDistance(
  const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
  double target_distance) const
{
  if (path.poses.empty() || cumulative_distances.empty()) {
    return geometry_msgs::msg::PoseStamped();
  }
  if (target_distance <= 0.0) {
    return path.poses.front();
  }
  if (target_distance >= cumulative_distances.back()) {
    return path.poses.back();
  }
  auto it =
    std::lower_bound(cumulative_distances.begin(), cumulative_distances.end(), target_distance);
  size_t index = std::distance(cumulative_distances.begin(), it);

  if (index == 0) {
    return path.poses.front();
  }

  double ratio = (target_distance - cumulative_distances[index - 1]) /
                 (cumulative_distances[index] - cumulative_distances[index - 1]);
  geometry_msgs::msg::PoseStamped pose1 = path.poses[index - 1];
  geometry_msgs::msg::PoseStamped pose2 = path.poses[index];

  geometry_msgs::msg::PoseStamped interpolated_pose;
  interpolated_pose.header = pose2.header;
  interpolated_pose.pose.position.x =
    pose1.pose.position.x + ratio * (pose2.pose.position.x - pose1.pose.position.x);
  interpolated_pose.pose.position.y =
    pose1.pose.position.y + ratio * (pose2.pose.position.y - pose1.pose.position.y);
  interpolated_pose.pose.position.z =
    pose1.pose.position.z + ratio * (pose2.pose.position.z - pose1.pose.position.z);
  interpolated_pose.pose.orientation = pose2.pose.orientation;

  return interpolated_pose;
}

rcl_interfaces::msg::SetParametersResult OmniPidPursuitController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".translation_kp") {
        translation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".translation_ki") {
        translation_ki_ = parameter.as_double();
      } else if (name == plugin_name_ + ".translation_kd") {
        translation_kd_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_kp") {
        rotation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_ki") {
        rotation_ki_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_kd") {
        rotation_kd_ = parameter.as_double();
      } else if (name == plugin_name_ + ".transform_tolerance") {
        double transform_tolerance = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
      } else if (name == plugin_name_ + ".min_max_sum_error") {
        min_max_sum_error_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_dist") {
        lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_lookahead_dist") {
        min_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        max_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        lookahead_time_ = parameter.as_double();
      } else if (name == plugin_name_ + ".use_rotate_to_heading_treshold") {
        use_rotate_to_heading_treshold_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
        min_approach_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".approach_velocity_scaling_dist") {
        approach_velocity_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_max") {
        v_linear_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_min") {
        v_linear_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_max") {
        v_angular_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_min") {
        v_angular_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_min") {
        curvature_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_max") {
        curvature_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".reduction_ratio_at_high_curvature") {
        reduction_ratio_at_high_curvature_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_forward_dist") {
        curvature_forward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_backward_dist") {
        curvature_backward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_velocity_scaling_factor_rate") {
        max_velocity_scaling_factor_rate_ = parameter.as_double();
      } else if (name == plugin_name_ + ".align_to_target_threshold_deg") {
        align_to_target_threshold_deg_ = parameter.as_double();
      } else if (name == plugin_name_ + ".align_to_target_tolerance_deg") {
        align_to_target_tolerance_deg_ = parameter.as_double();
      } else if (name == plugin_name_ + ".align_to_target_rotation_kp") {
        align_to_target_rotation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".align_to_target_disable_distance") {
        align_to_target_disable_distance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".align_to_target_debug_throttle_sec") {
        align_to_target_debug_throttle_sec_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_interpolation") {
        use_interpolation_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_rotate_to_heading") {
        use_rotate_to_heading_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".align_to_target_before_motion") {
        align_to_target_before_motion_ = parameter.as_bool();
        if (!align_to_target_before_motion_) {
          align_to_target_active_ = false;
        }
      } else if (name == plugin_name_ + ".align_to_target_use_rear_heading") {
        align_to_target_use_rear_heading_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".align_to_target_debug") {
        align_to_target_debug_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

};  // namespace pb_omni_pid_pursuit_controller
// Register this controller as a nav2_core plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pb_omni_pid_pursuit_controller::OmniPidPursuitController, nav2_core::Controller)
