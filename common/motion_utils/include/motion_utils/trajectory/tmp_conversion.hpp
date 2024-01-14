// Copyright 2021 Tier IV, Inc.
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

#ifndef MOTION_UTILS__TRAJECTORY__TMP_CONVERSION_HPP_
#define MOTION_UTILS__TRAJECTORY__TMP_CONVERSION_HPP_

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/geometry/pose_deviation.hpp"

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <vector>

namespace motion_utils
{
using TrajectoryPoints = std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>;
/**
 * @brief Convert std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> to
 * autoware_auto_planning_msgs::msg::Trajectory. This function is temporarily added for porting to
 * autoware_auto_msgs. We should consider whether to remove this function after the porting is done.
 * @attention This function just clips
 * std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> up to the capacity of Trajectory.
 * Therefore, the error handling out of this function is necessary if the size of the input greater
 * than the capacity.
 * @todo Decide how to handle the situation that we need to use the trajectory with the size of
 * points larger than the capacity. (Tier IV)
 */
inline autoware_auto_planning_msgs::msg::Trajectory convertToTrajectory(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & trajectory)
{
  autoware_auto_planning_msgs::msg::Trajectory output{};
  for (const auto & pt : trajectory) {
    output.points.push_back(pt);
    if (output.points.size() >= output.CAPACITY) {
      break;
    }
  }
  return output;
}

/**
 * @brief Convert autoware_auto_planning_msgs::msg::Trajectory to
 * std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>.
 */
inline std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> convertToTrajectoryPointArray(
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory)
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> output(trajectory.points.size());
  std::copy(trajectory.points.begin(), trajectory.points.end(), output.begin());
  return output;
}

template <class T>
autoware_auto_planning_msgs::msg::Path convertToPath([[maybe_unused]] const T & input)
{
  static_assert(sizeof(T) == 0, "Only specializations of convertToPath can be used.");
  throw std::logic_error("Only specializations of convertToPath can be used.");
}

template <>
inline autoware_auto_planning_msgs::msg::Path convertToPath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input)
{
  autoware_auto_planning_msgs::msg::Path output{};
  output.header = input.header;
  output.left_bound = input.left_bound;
  output.right_bound = input.right_bound;
  output.points.resize(input.points.size());
  for (size_t i = 0; i < input.points.size(); ++i) {
    output.points.at(i) = input.points.at(i).point;
  }
  return output;
}

template <class T>
TrajectoryPoints convertToTrajectoryPoints([[maybe_unused]] const T & input)
{
  static_assert(sizeof(T) == 0, "Only specializations of convertToTrajectoryPoints can be used.");
  throw std::logic_error("Only specializations of convertToTrajectoryPoints can be used.");
}

template <>
inline TrajectoryPoints convertToTrajectoryPoints(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input)
{
  TrajectoryPoints output{};
  for (const auto & p : input.points) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint tp;
    tp.pose = p.point.pose;
    tp.longitudinal_velocity_mps = p.point.longitudinal_velocity_mps;
    // since path point doesn't have acc for now
    tp.acceleration_mps2 = 0;
    output.emplace_back(tp);
  }
  return output;
}

template <class T>
autoware_auto_planning_msgs::msg::PathWithLaneId convertToPathWithLaneId(
  [[maybe_unused]] const T & input)
{
  static_assert(sizeof(T) == 0, "Only specializations of convertToPathWithLaneId can be used.");
  throw std::logic_error("Only specializations of convertToPathWithLaneId can be used.");
}

template <>
inline autoware_auto_planning_msgs::msg::PathWithLaneId convertToPathWithLaneId(
  const TrajectoryPoints & input)
{
  autoware_auto_planning_msgs::msg::PathWithLaneId output{};
  for (const auto & p : input) {
    autoware_auto_planning_msgs::msg::PathPointWithLaneId pp;
    pp.point.pose = p.pose;
    pp.point.longitudinal_velocity_mps = p.longitudinal_velocity_mps;
    output.points.emplace_back(pp);
  }
  return output;
}

}  // namespace motion_utils

#endif  // MOTION_UTILS__TRAJECTORY__TMP_CONVERSION_HPP_
