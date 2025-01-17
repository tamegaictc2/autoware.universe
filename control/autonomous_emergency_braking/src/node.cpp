// Copyright 2022 TIER IV, Inc.
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

#include "autonomous_emergency_braking/node.hpp"

#include <pcl_ros/transforms.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <tier4_autoware_utils/ros/update_param.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/strategies/agnostic/hull_graham_andrew.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <tf2/utils.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace autoware::motion::control::autonomous_emergency_braking
{
using diagnostic_msgs::msg::DiagnosticStatus;
namespace bg = boost::geometry;

void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point)
{
  Point2d point;
  point.x() = geom_point.x;
  point.y() = geom_point.y;

  bg::append(polygon.outer(), point);
}

Polygon2d createPolygon(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & next_pose,
  const vehicle_info_util::VehicleInfo & vehicle_info, const double expand_width)
{
  Polygon2d polygon;

  const double longitudinal_offset = vehicle_info.max_longitudinal_offset_m;
  const double width = vehicle_info.vehicle_width_m / 2.0 + expand_width;
  const double rear_overhang = vehicle_info.rear_overhang_m;

  appendPointToPolygon(
    polygon,
    tier4_autoware_utils::calcOffsetPose(base_pose, longitudinal_offset, width, 0.0).position);
  appendPointToPolygon(
    polygon,
    tier4_autoware_utils::calcOffsetPose(base_pose, longitudinal_offset, -width, 0.0).position);
  appendPointToPolygon(
    polygon, tier4_autoware_utils::calcOffsetPose(base_pose, -rear_overhang, -width, 0.0).position);
  appendPointToPolygon(
    polygon, tier4_autoware_utils::calcOffsetPose(base_pose, -rear_overhang, width, 0.0).position);

  appendPointToPolygon(
    polygon,
    tier4_autoware_utils::calcOffsetPose(next_pose, longitudinal_offset, width, 0.0).position);
  appendPointToPolygon(
    polygon,
    tier4_autoware_utils::calcOffsetPose(next_pose, longitudinal_offset, -width, 0.0).position);
  appendPointToPolygon(
    polygon, tier4_autoware_utils::calcOffsetPose(next_pose, -rear_overhang, -width, 0.0).position);
  appendPointToPolygon(
    polygon, tier4_autoware_utils::calcOffsetPose(next_pose, -rear_overhang, width, 0.0).position);

  polygon = tier4_autoware_utils::isClockwise(polygon)
              ? polygon
              : tier4_autoware_utils::inverseClockwise(polygon);

  Polygon2d hull_polygon;
  bg::convex_hull(polygon, hull_polygon);

  return hull_polygon;
}

AEB::AEB(const rclcpp::NodeOptions & node_options)
: Node("AEB", node_options),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo()),
  collision_data_keeper_(this->get_clock())
{
  // Subscribers
  {
    sub_point_cloud_ = this->create_subscription<PointCloud2>(
      "~/input/pointcloud", rclcpp::SensorDataQoS(),
      std::bind(&AEB::onPointCloud, this, std::placeholders::_1));

    sub_velocity_ = this->create_subscription<VelocityReport>(
      "~/input/velocity", rclcpp::QoS{1}, std::bind(&AEB::onVelocity, this, std::placeholders::_1));

    sub_imu_ = this->create_subscription<Imu>(
      "~/input/imu", rclcpp::QoS{1}, std::bind(&AEB::onImu, this, std::placeholders::_1));

    sub_predicted_traj_ = this->create_subscription<Trajectory>(
      "~/input/predicted_trajectory", rclcpp::QoS{1},
      std::bind(&AEB::onPredictedTrajectory, this, std::placeholders::_1));

    sub_autoware_state_ = this->create_subscription<AutowareState>(
      "/autoware/state", rclcpp::QoS{1},
      std::bind(&AEB::onAutowareState, this, std::placeholders::_1));
  }

  // Publisher
  {
    pub_obstacle_pointcloud_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/obstacle_pointcloud", 1);
    debug_ego_path_publisher_ = this->create_publisher<MarkerArray>("~/debug/markers", 1);
  }
  // Diagnostics
  {
    updater_.setHardwareID("autonomous_emergency_braking");
    updater_.add("aeb_emergency_stop", this, &AEB::onCheckCollision);
  }
  // parameter
  publish_debug_pointcloud_ = declare_parameter<bool>("publish_debug_pointcloud");
  use_predicted_trajectory_ = declare_parameter<bool>("use_predicted_trajectory");
  use_imu_path_ = declare_parameter<bool>("use_imu_path");
  path_footprint_extra_margin_ = declare_parameter<double>("path_footprint_extra_margin");
  detection_range_min_height_ = declare_parameter<double>("detection_range_min_height");
  detection_range_max_height_margin_ =
    declare_parameter<double>("detection_range_max_height_margin");
  voxel_grid_x_ = declare_parameter<double>("voxel_grid_x");
  voxel_grid_y_ = declare_parameter<double>("voxel_grid_y");
  voxel_grid_z_ = declare_parameter<double>("voxel_grid_z");
  min_generated_path_length_ = declare_parameter<double>("min_generated_path_length");
  expand_width_ = declare_parameter<double>("expand_width");
  longitudinal_offset_ = declare_parameter<double>("longitudinal_offset");
  t_response_ = declare_parameter<double>("t_response");
  a_ego_min_ = declare_parameter<double>("a_ego_min");
  a_obj_min_ = declare_parameter<double>("a_obj_min");

  cluster_tolerance_ = declare_parameter<double>("cluster_tolerance");
  minimum_cluster_size_ = declare_parameter<int>("minimum_cluster_size");
  maximum_cluster_size_ = declare_parameter<int>("maximum_cluster_size");

  imu_prediction_time_horizon_ = declare_parameter<double>("imu_prediction_time_horizon");
  imu_prediction_time_interval_ = declare_parameter<double>("imu_prediction_time_interval");
  mpc_prediction_time_horizon_ = declare_parameter<double>("mpc_prediction_time_horizon");
  mpc_prediction_time_interval_ = declare_parameter<double>("mpc_prediction_time_interval");

  {  // Object history data keeper setup
    const auto previous_obstacle_keep_time =
      declare_parameter<double>("previous_obstacle_keep_time");
    const auto collision_keeping_sec = declare_parameter<double>("collision_keeping_sec");
    collision_data_keeper_.setTimeout(collision_keeping_sec, previous_obstacle_keep_time);
  }

  // Parameter Callback
  set_param_res_ =
    add_on_set_parameters_callback(std::bind(&AEB::onParameter, this, std::placeholders::_1));

  // start time
  const double aeb_hz = declare_parameter<double>("aeb_hz");
  const auto period_ns = rclcpp::Rate(aeb_hz).period();
  timer_ = rclcpp::create_timer(this, this->get_clock(), period_ns, std::bind(&AEB::onTimer, this));
}

rcl_interfaces::msg::SetParametersResult AEB::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;
  updateParam<bool>(parameters, "publish_debug_pointcloud", publish_debug_pointcloud_);
  updateParam<bool>(parameters, "use_predicted_trajectory", use_predicted_trajectory_);
  updateParam<bool>(parameters, "use_imu_path", use_imu_path_);
  updateParam<double>(parameters, "path_footprint_extra_margin", path_footprint_extra_margin_);
  updateParam<double>(parameters, "detection_range_min_height", detection_range_min_height_);
  updateParam<double>(
    parameters, "detection_range_max_height_margin", detection_range_max_height_margin_);
  updateParam<double>(parameters, "voxel_grid_x", voxel_grid_x_);
  updateParam<double>(parameters, "voxel_grid_y", voxel_grid_y_);
  updateParam<double>(parameters, "voxel_grid_z", voxel_grid_z_);
  updateParam<double>(parameters, "min_generated_path_length", min_generated_path_length_);
  updateParam<double>(parameters, "expand_width", expand_width_);
  updateParam<double>(parameters, "longitudinal_offset", longitudinal_offset_);
  updateParam<double>(parameters, "t_response", t_response_);
  updateParam<double>(parameters, "a_ego_min", a_ego_min_);
  updateParam<double>(parameters, "a_obj_min", a_obj_min_);

  updateParam<double>(parameters, "cluster_tolerance", cluster_tolerance_);
  updateParam<int>(parameters, "minimum_cluster_size", minimum_cluster_size_);
  updateParam<int>(parameters, "maximum_cluster_size", maximum_cluster_size_);

  updateParam<double>(parameters, "imu_prediction_time_horizon", imu_prediction_time_horizon_);
  updateParam<double>(parameters, "imu_prediction_time_interval", imu_prediction_time_interval_);
  updateParam<double>(parameters, "mpc_prediction_time_horizon", mpc_prediction_time_horizon_);
  updateParam<double>(parameters, "mpc_prediction_time_interval", mpc_prediction_time_interval_);

  {  // Object history data keeper setup
    auto [previous_obstacle_keep_time, collision_keeping_sec] = collision_data_keeper_.getTimeout();
    updateParam<double>(parameters, "previous_obstacle_keep_time", previous_obstacle_keep_time);
    updateParam<double>(parameters, "collision_keeping_sec", collision_keeping_sec);
    collision_data_keeper_.setTimeout(collision_keeping_sec, previous_obstacle_keep_time);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void AEB::onTimer()
{
  updater_.force_update();
}

void AEB::onVelocity(const VelocityReport::ConstSharedPtr input_msg)
{
  current_velocity_ptr_ = input_msg;
}

void AEB::onImu(const Imu::ConstSharedPtr input_msg)
{
  // transform imu
  geometry_msgs::msg::TransformStamped transform_stamped{};
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      "base_link", input_msg->header.frame_id, input_msg->header.stamp,
      rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "[AEB] Failed to look up transform from base_link to" << input_msg->header.frame_id);
    return;
  }

  angular_velocity_ptr_ = std::make_shared<Vector3>();
  tf2::doTransform(input_msg->angular_velocity, *angular_velocity_ptr_, transform_stamped);
}

void AEB::onPredictedTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr input_msg)
{
  predicted_traj_ptr_ = input_msg;
}

void AEB::onAutowareState(const AutowareState::ConstSharedPtr input_msg)
{
  autoware_state_ = input_msg;
}

void AEB::onPointCloud(const PointCloud2::ConstSharedPtr input_msg)
{
  PointCloud::Ptr pointcloud_ptr(new PointCloud);
  pcl::fromROSMsg(*input_msg, *pointcloud_ptr);

  if (input_msg->header.frame_id != "base_link") {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "[AEB]: Input point cloud frame is not base_link and it is " << input_msg->header.frame_id);
    // transform pointcloud
    geometry_msgs::msg::TransformStamped transform_stamped{};
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        "base_link", input_msg->header.frame_id, input_msg->header.stamp,
        rclcpp::Duration::from_seconds(0.5));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "[AEB] Failed to look up transform from base_link to" << input_msg->header.frame_id);
      return;
    }

    // transform by using eigen matrix
    PointCloud2 transformed_points{};
    const Eigen::Matrix4f affine_matrix =
      tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(affine_matrix, *input_msg, transformed_points);
    pcl::fromROSMsg(transformed_points, *pointcloud_ptr);
  }

  // apply z-axis filter for removing False Positive points
  PointCloud::Ptr height_filtered_pointcloud_ptr(new PointCloud);
  pcl::PassThrough<pcl::PointXYZ> height_filter;
  height_filter.setInputCloud(pointcloud_ptr);
  height_filter.setFilterFieldName("z");
  height_filter.setFilterLimits(
    detection_range_min_height_,
    vehicle_info_.vehicle_height_m + detection_range_max_height_margin_);
  height_filter.filter(*height_filtered_pointcloud_ptr);

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  PointCloud::Ptr no_height_filtered_pointcloud_ptr(new PointCloud);
  filter.setInputCloud(height_filtered_pointcloud_ptr);
  filter.setLeafSize(voxel_grid_x_, voxel_grid_y_, voxel_grid_z_);
  filter.filter(*no_height_filtered_pointcloud_ptr);

  obstacle_ros_pointcloud_ptr_ = std::make_shared<PointCloud2>();

  pcl::toROSMsg(*no_height_filtered_pointcloud_ptr, *obstacle_ros_pointcloud_ptr_);
  obstacle_ros_pointcloud_ptr_->header = input_msg->header;
}

bool AEB::isDataReady()
{
  const auto missing = [this](const auto & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "[AEB] waiting for %s", name);
    return false;
  };

  if (!current_velocity_ptr_) {
    return missing("ego velocity");
  }

  if (!obstacle_ros_pointcloud_ptr_) {
    return missing("object pointcloud");
  }

  if (use_imu_path_ && !angular_velocity_ptr_) {
    return missing("imu");
  }

  if (use_predicted_trajectory_ && !predicted_traj_ptr_) {
    return missing("control predicted trajectory");
  }

  if (!autoware_state_) {
    return missing("autoware_state");
  }

  return true;
}

void AEB::onCheckCollision(DiagnosticStatusWrapper & stat)
{
  MarkerArray debug_markers;
  checkCollision(debug_markers);

  if (!collision_data_keeper_.checkCollisionExpired()) {
    const std::string error_msg = "[AEB]: Emergency Brake";
    const auto diag_level = DiagnosticStatus::ERROR;
    stat.summary(diag_level, error_msg);
    const auto & data = collision_data_keeper_.get();
    stat.addf("RSS", "%.2f", data.rss);
    stat.addf("Distance", "%.2f", data.distance_to_object);
    stat.addf("Object Speed", "%.2f", data.velocity);
    addCollisionMarker(data, debug_markers);
  } else {
    const std::string error_msg = "[AEB]: No Collision";
    const auto diag_level = DiagnosticStatus::OK;
    stat.summary(diag_level, error_msg);
  }

  // publish debug markers
  debug_ego_path_publisher_->publish(debug_markers);
}

bool AEB::checkCollision(MarkerArray & debug_markers)
{
  using colorTuple = std::tuple<double, double, double, double>;

  // step1. check data
  if (!isDataReady()) {
    return false;
  }

  // if not driving, disable aeb
  if (autoware_state_->state != AutowareState::DRIVING) {
    return false;
  }

  // step2. create velocity data check if the vehicle stops or not
  const double current_v = current_velocity_ptr_->longitudinal_velocity;
  if (current_v < 0.1) {
    return false;
  }

  auto check_collision = [&](
                           const auto & path, const colorTuple & debug_colors,
                           const std::string & debug_ns,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_objects) {
    // Crop out Pointcloud using an extra wide ego path
    const auto expanded_ego_polys =
      generatePathFootprint(path, expand_width_ + path_footprint_extra_margin_);
    cropPointCloudWithEgoFootprintPath(expanded_ego_polys, filtered_objects);

    // Check which points of the cropped point cloud are on the ego path, and get the closest one
    std::vector<ObjectData> objects_from_point_clusters;
    const auto ego_polys = generatePathFootprint(path, expand_width_);
    const auto current_time = obstacle_ros_pointcloud_ptr_->header.stamp;
    createObjectDataUsingPointCloudClusters(
      path, ego_polys, current_time, objects_from_point_clusters, filtered_objects);

    // Get only the closest object and calculate its speed
    const auto closest_object_point = std::invoke([&]() -> std::optional<ObjectData> {
      const auto closest_object_point_itr = std::min_element(
        objects_from_point_clusters.begin(), objects_from_point_clusters.end(),
        [](const auto & o1, const auto & o2) {
          return o1.distance_to_object < o2.distance_to_object;
        });

      if (closest_object_point_itr == objects_from_point_clusters.end()) {
        return std::nullopt;
      }
      const auto closest_object_speed = collision_data_keeper_.calcObjectSpeedFromHistory(
        *closest_object_point_itr, path, current_v);
      if (!closest_object_speed.has_value()) {
        return std::nullopt;
      }
      closest_object_point_itr->velocity = closest_object_speed.value();
      return std::make_optional<ObjectData>(*closest_object_point_itr);
    });

    // Add debug markers
    {
      const auto [color_r, color_g, color_b, color_a] = debug_colors;
      addMarker(
        this->get_clock()->now(), path, ego_polys, objects_from_point_clusters,
        closest_object_point, color_r, color_g, color_b, color_a, debug_ns, debug_markers);
    }
    // check collision using rss distance
    return (closest_object_point.has_value())
             ? hasCollision(current_v, closest_object_point.value())
             : false;
  };

  // step3. make function to check collision with ego path created with sensor data
  const auto has_collision_ego = [&](pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_objects) -> bool {
    if (!use_imu_path_) return false;
    const double current_w = angular_velocity_ptr_->z;
    constexpr colorTuple debug_color = {0.0 / 256.0, 148.0 / 256.0, 205.0 / 256.0, 0.999};
    const std::string ns = "ego";
    const auto ego_path = generateEgoPath(current_v, current_w);

    return check_collision(ego_path, debug_color, ns, filtered_objects);
  };

  // step4. make function to check collision with predicted trajectory from control module
  const auto has_collision_predicted =
    [&](pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_objects) -> bool {
    if (!use_predicted_trajectory_ || !predicted_traj_ptr_) return false;
    const auto predicted_traj_ptr = predicted_traj_ptr_;
    const auto predicted_path_opt = generateEgoPath(*predicted_traj_ptr);

    if (!predicted_path_opt) return false;
    constexpr colorTuple debug_color = {0.0 / 256.0, 100.0 / 256.0, 0.0 / 256.0, 0.999};
    const std::string ns = "predicted";
    const auto & predicted_path = predicted_path_opt.value();

    return check_collision(predicted_path, debug_color, ns, filtered_objects);
  };

  // Data of filtered point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_objects(new PointCloud);
  // evaluate if there is a collision for both paths
  const bool has_collision =
    has_collision_ego(filtered_objects) || has_collision_predicted(filtered_objects);

  // Debug print
  if (!filtered_objects->empty() && publish_debug_pointcloud_) {
    const auto filtered_objects_ros_pointcloud_ptr_ = std::make_shared<PointCloud2>();
    pcl::toROSMsg(*filtered_objects, *filtered_objects_ros_pointcloud_ptr_);
    pub_obstacle_pointcloud_->publish(*filtered_objects_ros_pointcloud_ptr_);
  }
  return has_collision;
}

bool AEB::hasCollision(const double current_v, const ObjectData & closest_object)
{
  const double & obj_v = closest_object.velocity;
  const double & t = t_response_;
  const double rss_dist = current_v * t + (current_v * current_v) / (2 * std::fabs(a_ego_min_)) -
                          obj_v * obj_v / (2 * std::fabs(a_obj_min_)) + longitudinal_offset_;
  if (closest_object.distance_to_object < rss_dist) {
    // collision happens
    ObjectData collision_data = closest_object;
    collision_data.rss = rss_dist;
    collision_data.distance_to_object = closest_object.distance_to_object;
    collision_data_keeper_.setCollisionData(collision_data);
    return true;
  }
  return false;
}

Path AEB::generateEgoPath(const double curr_v, const double curr_w)
{
  Path path;
  double curr_x = 0.0;
  double curr_y = 0.0;
  double curr_yaw = 0.0;
  geometry_msgs::msg::Pose ini_pose;
  ini_pose.position = tier4_autoware_utils::createPoint(curr_x, curr_y, 0.0);
  ini_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(curr_yaw);
  path.push_back(ini_pose);

  if (curr_v < 0.1) {
    // if current velocity is too small, assume it stops at the same point
    return path;
  }

  constexpr double epsilon = 1e-6;
  const double & dt = imu_prediction_time_interval_;
  const double & horizon = imu_prediction_time_horizon_;
  for (double t = 0.0; t < horizon + epsilon; t += dt) {
    curr_x = curr_x + curr_v * std::cos(curr_yaw) * dt;
    curr_y = curr_y + curr_v * std::sin(curr_yaw) * dt;
    curr_yaw = curr_yaw + curr_w * dt;
    geometry_msgs::msg::Pose current_pose;
    current_pose.position = tier4_autoware_utils::createPoint(curr_x, curr_y, 0.0);
    current_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(curr_yaw);
    if (tier4_autoware_utils::calcDistance2d(path.back(), current_pose) < 1e-3) {
      continue;
    }
    path.push_back(current_pose);
  }

  // If path is shorter than minimum path length
  while (motion_utils::calcArcLength(path) < min_generated_path_length_) {
    curr_x = curr_x + curr_v * std::cos(curr_yaw) * dt;
    curr_y = curr_y + curr_v * std::sin(curr_yaw) * dt;
    curr_yaw = curr_yaw + curr_w * dt;
    geometry_msgs::msg::Pose current_pose;
    current_pose.position = tier4_autoware_utils::createPoint(curr_x, curr_y, 0.0);
    current_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(curr_yaw);
    if (tier4_autoware_utils::calcDistance2d(path.back(), current_pose) < 1e-3) {
      continue;
    }
    path.push_back(current_pose);
  }
  return path;
}

std::optional<Path> AEB::generateEgoPath(const Trajectory & predicted_traj)
{
  if (predicted_traj.points.empty()) {
    return std::nullopt;
  }

  geometry_msgs::msg::TransformStamped transform_stamped{};
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      "base_link", predicted_traj.header.frame_id, predicted_traj.header.stamp,
      rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "[AEB] Failed to look up transform from base_link to map");
    return std::nullopt;
  }

  // create path
  Path path;
  path.resize(predicted_traj.points.size());
  for (size_t i = 0; i < predicted_traj.points.size(); ++i) {
    geometry_msgs::msg::Pose map_pose;
    tf2::doTransform(predicted_traj.points.at(i).pose, map_pose, transform_stamped);
    path.at(i) = map_pose;

    if (i * mpc_prediction_time_interval_ > mpc_prediction_time_horizon_) {
      break;
    }
  }
  return path;
}

std::vector<Polygon2d> AEB::generatePathFootprint(
  const Path & path, const double extra_width_margin)
{
  std::vector<Polygon2d> polygons;
  for (size_t i = 0; i < path.size() - 1; ++i) {
    polygons.push_back(
      createPolygon(path.at(i), path.at(i + 1), vehicle_info_, extra_width_margin));
  }
  return polygons;
}

void AEB::createObjectDataUsingPointCloudClusters(
  const Path & ego_path, const std::vector<Polygon2d> & ego_polys, const rclcpp::Time & stamp,
  std::vector<ObjectData> & objects, const pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_points_ptr)
{
  // check if the predicted path has valid number of points
  if (ego_path.size() < 2 || ego_polys.empty() || obstacle_points_ptr->empty()) {
    return;
  }

  // eliminate noisy points by only considering points belonging to clusters of at least a certain
  // size
  const std::vector<pcl::PointIndices> cluster_indices = std::invoke([&]() {
    std::vector<pcl::PointIndices> cluster_idx;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(obstacle_points_ptr);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(minimum_cluster_size_);
    ec.setMaxClusterSize(maximum_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(obstacle_points_ptr);
    ec.extract(cluster_idx);
    return cluster_idx;
  });

  PointCloud::Ptr points_belonging_to_cluster_hulls(new PointCloud);
  for (const auto & indices : cluster_indices) {
    PointCloud::Ptr cluster(new PointCloud);
    for (const auto & index : indices.indices) {
      cluster->push_back((*obstacle_points_ptr)[index]);
    }
    // Make a 2d convex hull for the objects
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setDimension(2);
    hull.setInputCloud(cluster);
    std::vector<pcl::Vertices> polygons;
    PointCloud::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*surface_hull, polygons);
    for (const auto & p : *surface_hull) {
      points_belonging_to_cluster_hulls->push_back(p);
    }
  }

  // select points inside the ego footprint path
  const auto current_p = tier4_autoware_utils::createPoint(
    ego_path[0].position.x, ego_path[0].position.y, ego_path[0].position.z);

  for (const auto & p : *points_belonging_to_cluster_hulls) {
    const auto obj_position = tier4_autoware_utils::createPoint(p.x, p.y, p.z);
    const double dist_ego_to_object =
      motion_utils::calcSignedArcLength(ego_path, current_p, obj_position) -
      vehicle_info_.max_longitudinal_offset_m;
    // objects behind ego are ignored
    if (dist_ego_to_object < 0.0) continue;

    ObjectData obj;
    obj.stamp = stamp;
    obj.position = obj_position;
    obj.velocity = 0.0;
    obj.distance_to_object = dist_ego_to_object;

    const Point2d obj_point(p.x, p.y);
    for (const auto & ego_poly : ego_polys) {
      if (bg::within(obj_point, ego_poly)) {
        objects.push_back(obj);
        break;
      }
    }
  }
}

void AEB::cropPointCloudWithEgoFootprintPath(
  const std::vector<Polygon2d> & ego_polys, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_objects)
{
  PointCloud::Ptr full_points_ptr(new PointCloud);
  pcl::fromROSMsg(*obstacle_ros_pointcloud_ptr_, *full_points_ptr);
  // Create a Point cloud with the points of the ego footprint
  PointCloud::Ptr path_polygon_points(new PointCloud);
  std::for_each(ego_polys.begin(), ego_polys.end(), [&](const auto & poly) {
    std::for_each(poly.outer().begin(), poly.outer().end(), [&](const auto & p) {
      pcl::PointXYZ point(p.x(), p.y(), 0.0);
      path_polygon_points->push_back(point);
    });
  });
  // Make a surface hull with the ego footprint to filter out points
  pcl::ConvexHull<pcl::PointXYZ> hull;
  hull.setDimension(2);
  hull.setInputCloud(path_polygon_points);
  std::vector<pcl::Vertices> polygons;
  pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
  hull.reconstruct(*surface_hull, polygons);
  // Filter out points outside of the path's convex hull
  pcl::CropHull<pcl::PointXYZ> path_polygon_hull_filter;
  path_polygon_hull_filter.setDim(2);
  path_polygon_hull_filter.setInputCloud(full_points_ptr);
  path_polygon_hull_filter.setHullIndices(polygons);
  path_polygon_hull_filter.setHullCloud(surface_hull);
  path_polygon_hull_filter.filter(*filtered_objects);
  filtered_objects->header.stamp = full_points_ptr->header.stamp;
}

void AEB::addMarker(
  const rclcpp::Time & current_time, const Path & path, const std::vector<Polygon2d> & polygons,
  const std::vector<ObjectData> & objects, const std::optional<ObjectData> & closest_object,
  const double color_r, const double color_g, const double color_b, const double color_a,
  const std::string & ns, MarkerArray & debug_markers)
{
  auto path_marker = tier4_autoware_utils::createDefaultMarker(
    "base_link", current_time, ns + "_path", 0L, Marker::LINE_STRIP,
    tier4_autoware_utils::createMarkerScale(0.2, 0.2, 0.2),
    tier4_autoware_utils::createMarkerColor(color_r, color_g, color_b, color_a));
  path_marker.points.resize(path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    path_marker.points.at(i) = path.at(i).position;
  }
  debug_markers.markers.push_back(path_marker);

  auto polygon_marker = tier4_autoware_utils::createDefaultMarker(
    "base_link", current_time, ns + "_polygon", 0, Marker::LINE_LIST,
    tier4_autoware_utils::createMarkerScale(0.03, 0.0, 0.0),
    tier4_autoware_utils::createMarkerColor(color_r, color_g, color_b, color_a));
  for (const auto & poly : polygons) {
    for (size_t dp_idx = 0; dp_idx < poly.outer().size(); ++dp_idx) {
      const auto & boost_cp = poly.outer().at(dp_idx);
      const auto & boost_np = poly.outer().at((dp_idx + 1) % poly.outer().size());
      const auto curr_point = tier4_autoware_utils::createPoint(boost_cp.x(), boost_cp.y(), 0.0);
      const auto next_point = tier4_autoware_utils::createPoint(boost_np.x(), boost_np.y(), 0.0);
      polygon_marker.points.push_back(curr_point);
      polygon_marker.points.push_back(next_point);
    }
  }
  debug_markers.markers.push_back(polygon_marker);

  auto object_data_marker = tier4_autoware_utils::createDefaultMarker(
    "base_link", current_time, ns + "_objects", 0, Marker::SPHERE_LIST,
    tier4_autoware_utils::createMarkerScale(0.5, 0.5, 0.5),
    tier4_autoware_utils::createMarkerColor(color_r, color_g, color_b, color_a));
  for (const auto & e : objects) {
    object_data_marker.points.push_back(e.position);
  }
  debug_markers.markers.push_back(object_data_marker);

  // Visualize planner type text
  if (closest_object.has_value()) {
    const auto & obj = closest_object.value();
    const auto color = tier4_autoware_utils::createMarkerColor(0.95, 0.95, 0.95, 0.999);
    auto closest_object_velocity_marker_array = tier4_autoware_utils::createDefaultMarker(
      "base_link", obj.stamp, ns + "_closest_object_velocity", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      tier4_autoware_utils::createMarkerScale(0.0, 0.0, 0.7), color);
    closest_object_velocity_marker_array.pose.position = obj.position;
    const auto ego_velocity = current_velocity_ptr_->longitudinal_velocity;
    closest_object_velocity_marker_array.text =
      "Object velocity: " + std::to_string(obj.velocity) + " [m/s]\n";
    closest_object_velocity_marker_array.text +=
      "Object relative velocity to ego: " + std::to_string(obj.velocity - ego_velocity) + " [m/s]";
    debug_markers.markers.push_back(closest_object_velocity_marker_array);
  }
}

void AEB::addCollisionMarker(const ObjectData & data, MarkerArray & debug_markers)
{
  auto point_marker = tier4_autoware_utils::createDefaultMarker(
    "base_link", data.stamp, "collision_point", 0, Marker::SPHERE,
    tier4_autoware_utils::createMarkerScale(0.3, 0.3, 0.3),
    tier4_autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.3));
  point_marker.pose.position = data.position;
  debug_markers.markers.push_back(point_marker);
}

}  // namespace autoware::motion::control::autonomous_emergency_braking

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::autonomous_emergency_braking::AEB)
