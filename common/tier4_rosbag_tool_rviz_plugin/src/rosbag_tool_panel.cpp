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

#include "rosbag_tool_panel.hpp"

#include <rclcpp/rclcpp.hpp>

#include <ctime>
#include <cstdlib>
#include <filesystem>
#include <iostream>

using std::placeholders::_1;
using std::placeholders::_2;

void setFormatDate(QLabel * line, double time)
{
  char buffer[128];
  auto seconds = static_cast<time_t>(time);
  strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H-%M-%S", localtime(&seconds));
  line->setText(QString("- ") + QString(buffer));
}

RosbagTooPanel::RosbagTooPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  std::filesystem::create_directory("rosbag_record");

  auto * cap_layout = new QHBoxLayout;
  {
    record_button_ptr_ = new QPushButton("Record Start");
    record_button_ptr_->setFixedWidth(300);
    connect(record_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickRosbagRecord()));
    cap_layout->addWidget(record_button_ptr_);
    ros_time_label_ = new QLabel;
    cap_layout->addWidget(ros_time_label_);
    // initialize file name system clock is better for identification.
    setFormatDate(ros_time_label_, rclcpp::Clock().now().seconds());
    setLayout(cap_layout);
  }

  state_ = State::WAITING_FOR_RECORD;

  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &RosbagTooPanel::update);
  timer->start(1000);
}

void RosbagTooPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

void RosbagTooPanel::onClickRosbagRecord()
{
  switch (state_) {
    case State::WAITING_FOR_RECORD:
      // initialize setting
      record_button_ptr_->setText("Recrding...");
      record_button_ptr_->setStyleSheet("background-color: #FF0000;");
      writer_->open("/home/emb4/rosbag");
      state_ = State::RECORDING;
      break;
    case State::RECORDING:
      record_button_ptr_->setText("Record Finish");
      record_button_ptr_->setStyleSheet("background-color: #00FF00;");
      writer_->close();
      state_ = State::WAITING_FOR_RECORD;
      break;
  }
}

void RosbagTooPanel::update()
{
  setFormatDate(ros_time_label_, rclcpp::Clock().now().seconds());
}

RosbagTooPanel::~RosbagTooPanel() = default;

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(RosbagTooPanel, rviz_common::Panel)
