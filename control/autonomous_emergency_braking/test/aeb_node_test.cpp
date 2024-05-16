// Copyright 2024 Tier IV, Inc.
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

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>

using autoware::motion::control::autonomous_emergency_braking::AEB;
using std::chrono_literals;

class AEBTest : public ::testing::Test
{
protected:
  std::shared_ptr<AEB> aeb_node_;

  virtual void SetUp()
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    aeb_node_ = std::make_shared<AEB>(options);
  }

  virtual void TearDown() { rclcpp::shutdown(); }
};

TEST_F(AEBTest, TestSubscriptions)
{
  // Ensure that the node is correctly initialized and subscribing to topics
  ASSERT_NE(aeb_node_->sub_point_cloud_, nullptr);
  ASSERT_NE(aeb_node_->sub_velocity_, nullptr);
  ASSERT_NE(aeb_node_->sub_imu_, nullptr);
  ASSERT_NE(aeb_node_->sub_predicted_traj_, nullptr);
  ASSERT_NE(aeb_node_->sub_autoware_state_, nullptr);
}

TEST_F(AEBTest, TestPublishers)
{
  // Ensure that the node is correctly initialized and publishing to topics
  ASSERT_NE(aeb_node_->pub_obstacle_pointcloud_, nullptr);
  ASSERT_NE(aeb_node_->debug_ego_path_publisher_, nullptr);
}

TEST_F(AEBTest, TestIsDataReady)
{
  // Initially, data should not be ready
  EXPECT_FALSE(aeb_node_->isDataReady());

  // Simulate incoming data
  auto velocity_msg = std::make_shared<autoware_auto_vehicle_msgs::msg::VelocityReport>();
  velocity_msg->header.stamp = rclcpp::Clock().now();
  velocity_msg->longitudinal_velocity = 10.0;
  aeb_node_->onVelocity(velocity_msg);

  auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
  imu_msg->header.stamp = rclcpp::Clock().now();
  aeb_node_->onImu(imu_msg);

  auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pointcloud_msg->header.stamp = rclcpp::Clock().now();
  aeb_node_->onPointCloud(pointcloud_msg);

  EXPECT_TRUE(aeb_node_->isDataReady());
}

TEST_F(AEBTest, TestCollisionDetection)
{
  MarkerArray debug_markers;
  double current_velocity = 10.0;
  ObjectData closest_object;
  closest_object.position.x = 10.0;
  closest_object.position.y = 0.0;
  closest_object.velocity = 0.0;
  closest_object.rss = 5.0;
  closest_object.distance_to_object = 15.0;

  // Simulate collision detection
  bool collision = aeb_node_->hasCollision(current_velocity, closest_object);
  EXPECT_TRUE(collision);
}

TEST_F(AEBTest, TestGenerateEgoPath)
{
  double curr_v = 10.0;
  double curr_w = 0.1;

  // Generate ego path
  auto path = aeb_node_->generateEgoPath(curr_v, curr_w);

  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path.size(), static_cast<size_t>(10));  // Based on default params
}

TEST_F(AEBTest, TestGeneratePathFootprint)
{
  Path path;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  path.push_back(pose);

  double extra_width_margin = 0.5;
  auto footprints = aeb_node_->generatePathFootprint(path, extra_width_margin);

  ASSERT_FALSE(footprints.empty());
  EXPECT_EQ(footprints.size(), static_cast<size_t>(1));  // One footprint for one pose
}

TEST_F(AEBTest, TestCalcObjectSpeedFromHistory)
{
  ObjectData closest_object;
  closest_object.position.x = 0.0;
  closest_object.position.y = 0.0;
  closest_object.stamp = rclcpp::Clock().now();

  Path path;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  path.push_back(pose);

  double current_ego_speed = 10.0;
  auto speed = aeb_node_->calcObjectSpeedFromHistory(closest_object, path, current_ego_speed);

  EXPECT_FALSE(speed.has_value());  // No history available yet

  aeb_node_->collision_data_keeper_.setPreviousObjectData(closest_object);
  closest_object.position.x = 1.0;
  closest_object.stamp = rclcpp::Clock().now() + 1s;

  speed = aeb_node_->calcObjectSpeedFromHistory(closest_object, path, current_ego_speed);

  EXPECT_TRUE(speed.has_value());
  EXPECT_DOUBLE_EQ(speed.value(), 1.0);  // Speed calculation from history
}
