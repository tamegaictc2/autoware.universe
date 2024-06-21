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

#ifndef ROSBAG_TOOL_PANEL_HPP_
#define ROSBAG_TOOL_PANEL_HPP_

// Qt
#include <QApplication>
#include <QDesktopWidget>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QScreen>
#include <QSpinBox>
#include <QTimer>

// rviz
#include <opencv2/opencv.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>

// ros
#include <std_srvs/srv/trigger.hpp>
#include <rosbag2_transport/recorder.hpp>

#include <memory>
#include <string>
#include <vector>

class QLineEdit;

class RosbagTooPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RosbagTooPanel(QWidget * parent = nullptr);
  ~RosbagTooPanel() override;
  void update();
  void onInitialize() override;
  void onTimer();

public Q_SLOTS:
  void onClickRosbagRecord();

private:
  QLabel * ros_time_label_;
  QPushButton * record_button_ptr_;
  enum class State { WAITING_FOR_RECORD, RECORDING };
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  State state_;

protected:
  rclcpp::Node::SharedPtr raw_node_;
};

#endif  // ROSBAG_TOOL_PANEL_HPP_
