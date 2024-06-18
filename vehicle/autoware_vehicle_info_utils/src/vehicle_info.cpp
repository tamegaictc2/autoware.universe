// Copyright 2015-2021 Autoware Foundation
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

#include "autoware_vehicle_info_utils/vehicle_info.hpp"

namespace autoware::vehicle_info_utils
{
autoware_universe_utils::LinearRing2d VehicleInfo::createFootprint(const double margin) const
{
  return createFootprint(margin, margin);
}

autoware_universe_utils::LinearRing2d VehicleInfo::createFootprint(
  const double lat_margin, const double lon_margin) const
{
  using autoware_universe_utils::LinearRing2d;
  using autoware_universe_utils::Point2d;

  const double x_front = front_overhang_m + wheel_base_m + lon_margin;
  const double x_center = wheel_base_m / 2.0;
  const double x_rear = -(rear_overhang_m + lon_margin);
  const double y_left = wheel_tread_m / 2.0 + left_overhang_m + lat_margin;
  const double y_right = -(wheel_tread_m / 2.0 + right_overhang_m + lat_margin);

  LinearRing2d footprint;
  footprint.push_back(Point2d{x_front, y_left});
  footprint.push_back(Point2d{x_front, y_right});
  footprint.push_back(Point2d{x_center, y_right});
  footprint.push_back(Point2d{x_rear, y_right});
  footprint.push_back(Point2d{x_rear, y_left});
  footprint.push_back(Point2d{x_center, y_left});
  footprint.push_back(Point2d{x_front, y_left});

  return footprint;
}

VehicleInfo createVehicleInfo(
  const double wheel_radius_m, const double wheel_width_m, const double wheel_base_m,
  const double wheel_tread_m, const double front_overhang_m, const double rear_overhang_m,
  const double left_overhang_m, const double right_overhang_m, const double vehicle_height_m,
  const double max_steer_angle_rad)
{
  // Calculate derived parameters
  const double vehicle_length_m_ = front_overhang_m + wheel_base_m + rear_overhang_m;
  const double vehicle_width_m_ = wheel_tread_m + left_overhang_m + right_overhang_m;
  const double min_longitudinal_offset_m_ = -rear_overhang_m;
  const double max_longitudinal_offset_m_ = front_overhang_m + wheel_base_m;
  const double min_lateral_offset_m_ = -(wheel_tread_m / 2.0 + right_overhang_m);
  const double max_lateral_offset_m_ = wheel_tread_m / 2.0 + left_overhang_m;
  const double min_height_offset_m_ = 0.0;
  const double max_height_offset_m_ = vehicle_height_m;

  return VehicleInfo{
    // Base parameters
    wheel_radius_m,
    wheel_width_m,
    wheel_base_m,
    wheel_tread_m,
    front_overhang_m,
    rear_overhang_m,
    left_overhang_m,
    right_overhang_m,
    vehicle_height_m,
    max_steer_angle_rad,
    // Derived parameters
    vehicle_length_m_,
    vehicle_width_m_,
    min_longitudinal_offset_m_,
    max_longitudinal_offset_m_,
    min_lateral_offset_m_,
    max_lateral_offset_m_,
    min_height_offset_m_,
    max_height_offset_m_,
  };
}

double VehicleInfo::calcMaxCurvature() const
{
  if (std::abs(wheel_base_m) < 1e-6) {
    throw std::invalid_argument("wheel_base_m is 0.");
  }
  if (std::abs(max_steer_angle_rad) < 1e-6) {
    return std::numeric_limits<double>::max();
  }

  const double radius = wheel_base_m / std::tan(max_steer_angle_rad);
  const double curvature = 1.0 / radius;
  return curvature;
}
double VehicleInfo::calcCurvatureFromSteerAngle(const double steer_angle) const
{
  if (std::abs(wheel_base_m) < 1e-6) {
    throw std::invalid_argument("wheel_base_m is 0.");
  }
  if (std::abs(steer_angle) < 1e-6) {
    return std::numeric_limits<double>::max();
  }

  const double radius = wheel_base_m / std::tan(steer_angle);
  const double curvature = 1.0 / radius;
  return curvature;
}
}  // namespace autoware::vehicle_info_utils
