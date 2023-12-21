//
//  Copyright 2022 TIER IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#include "traffic_light_publish_panel.hpp"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QString>
#include <QStringList>
#include <QVBoxLayout>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <rviz_common/display_context.hpp>

#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#undef signals
namespace rviz_plugins
{
TrafficLightPublishPanel::TrafficLightPublishPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // Publish Rate
  publishing_rate_input_ = new QSpinBox();
  publishing_rate_input_->setRange(1, 100);
  publishing_rate_input_->setSingleStep(1);
  publishing_rate_input_->setValue(10);
  publishing_rate_input_->setSuffix("Hz");

  // Traffic Light ID
  traffic_light_id_input_ = new QComboBox();  // init items in first onVectorMap
  traffic_light_id_input_->view()->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  // Traffic Light Confidence
  traffic_light_confidence_input_ = new QDoubleSpinBox();
  traffic_light_confidence_input_->setRange(0.0, 1.0);
  traffic_light_confidence_input_->setSingleStep(0.1);
  traffic_light_confidence_input_->setValue(1.0);

  // Traffic Light Color
  light_color_combo_ = new QComboBox();
  light_color_combo_->addItems({"RED", "AMBER", "GREEN", "WHITE", "UNKNOWN"});

  // Traffic Light Shape
  light_shape_combo_ = new QComboBox();
  light_shape_combo_->addItems(
    {"CIRCLE", "LEFT_ARROW", "RIGHT_ARROW", "UP_ARROW", "DOWN_ARROW", "DOWN_LEFT_ARROW",
     "DOWN_RIGHT_ARROW", "CROSS", "UNKNOWN"});

  // Traffic Light Status
  light_status_combo_ = new QComboBox();
  light_status_combo_->addItems({"SOLID_ON", "SOLID_OFF", "FLASHING", "UNKNOWN"});

  // Set Traffic Signals Button
  set_button_ = new QPushButton("SET");

  // Reset Traffic Signals Button
  reset_button_ = new QPushButton("RESET");

  // Publish Traffic Signals Button
  publish_button_ = new QPushButton("PUBLISH");

  auto vertical_header = new QHeaderView(Qt::Vertical);
  vertical_header->hide();
  auto horizontal_header = new QHeaderView(Qt::Horizontal);
  horizontal_header->setSectionResizeMode(QHeaderView::Stretch);

  traffic_table_ = new QTableWidget();
  traffic_table_->setColumnCount(5);
  traffic_table_->setHorizontalHeaderLabels({"ID", "Color", "Shape", "Status", "Confidence"});
  traffic_table_->setVerticalHeader(vertical_header);
  traffic_table_->setHorizontalHeader(horizontal_header);

  connect(publishing_rate_input_, SIGNAL(valueChanged(int)), this, SLOT(onRateChanged(int)));
  connect(set_button_, SIGNAL(clicked()), SLOT(onSetTrafficLightState()));
  connect(reset_button_, SIGNAL(clicked()), SLOT(onResetTrafficLightState()));
  connect(publish_button_, SIGNAL(clicked()), SLOT(onPublishTrafficLightState()));

  auto * h_layout_1 = new QHBoxLayout;
  h_layout_1->addWidget(new QLabel("Rate: "));
  h_layout_1->addWidget(publishing_rate_input_);
  h_layout_1->addWidget(new QLabel("ID: "));
  h_layout_1->addWidget(traffic_light_id_input_);
  h_layout_1->addWidget(new QLabel("Confidence: "));
  h_layout_1->addWidget(traffic_light_confidence_input_);

  auto * h_layout_2 = new QHBoxLayout;
  h_layout_2->addWidget(new QLabel("Traffic Light Color: "), 40);
  h_layout_2->addWidget(light_color_combo_, 60);

  auto * h_layout_3 = new QHBoxLayout;
  h_layout_3->addWidget(new QLabel("Traffic Light Shape: "), 40);
  h_layout_3->addWidget(light_shape_combo_, 60);

  auto * h_layout_4 = new QHBoxLayout;
  h_layout_4->addWidget(new QLabel("Traffic Light Status: "), 40);
  h_layout_4->addWidget(light_status_combo_, 60);

  auto * v_layout = new QVBoxLayout;
  v_layout->addLayout(h_layout_1);
  v_layout->addLayout(h_layout_2);
  v_layout->addLayout(h_layout_3);
  v_layout->addLayout(h_layout_4);
  v_layout->addWidget(set_button_);
  v_layout->addWidget(reset_button_);
  v_layout->addWidget(publish_button_);

  auto * h_layout_5 = new QHBoxLayout;
  h_layout_5->addLayout(v_layout);
  h_layout_5->addWidget(traffic_table_);

  setLayout(h_layout_5);
}

void TrafficLightPublishPanel::onSetTrafficLightState()
{
  const auto traffic_light_id_str = traffic_light_id_input_->currentText();
  const auto traffic_light_id = std::stoi(traffic_light_id_str.toStdString());
  const auto color = light_color_combo_->currentText();
  const auto shape = light_shape_combo_->currentText();
  const auto status = light_status_combo_->currentText();

  TrafficSignalElement traffic_light;
  traffic_light.confidence = traffic_light_confidence_input_->value();

  if (color == "RED") {
    traffic_light.color = TrafficSignalElement::RED;
  } else if (color == "AMBER") {
    traffic_light.color = TrafficSignalElement::AMBER;
  } else if (color == "GREEN") {
    traffic_light.color = TrafficSignalElement::GREEN;
  } else if (color == "WHITE") {
    traffic_light.color = TrafficSignalElement::WHITE;
  } else if (color == "UNKNOWN") {
    traffic_light.color = TrafficSignalElement::UNKNOWN;
  }

  if (shape == "CIRCLE") {
    traffic_light.shape = TrafficSignalElement::CIRCLE;
  } else if (shape == "LEFT_ARROW") {
    traffic_light.shape = TrafficSignalElement::LEFT_ARROW;
  } else if (shape == "RIGHT_ARROW") {
    traffic_light.shape = TrafficSignalElement::RIGHT_ARROW;
  } else if (shape == "UP_ARROW") {
    traffic_light.shape = TrafficSignalElement::UP_ARROW;
  } else if (shape == "DOWN_ARROW") {
    traffic_light.shape = TrafficSignalElement::DOWN_ARROW;
  } else if (shape == "DOWN_LEFT_ARROW") {
    traffic_light.shape = TrafficSignalElement::DOWN_LEFT_ARROW;
  } else if (shape == "DOWN_RIGHT_ARROW") {
    traffic_light.shape = TrafficSignalElement::DOWN_RIGHT_ARROW;
  } else if (shape == "UNKNOWN") {
    traffic_light.shape = TrafficSignalElement::UNKNOWN;
  }

  if (status == "SOLID_OFF") {
    traffic_light.status = TrafficSignalElement::SOLID_OFF;
  } else if (status == "SOLID_ON") {
    traffic_light.status = TrafficSignalElement::SOLID_ON;
  } else if (status == "FLASHING") {
    traffic_light.status = TrafficSignalElement::FLASHING;
  } else if (status == "UNKNOWN") {
    traffic_light.status = TrafficSignalElement::UNKNOWN;
  }

  TrafficSignal traffic_signal;
  traffic_signal.elements.push_back(traffic_light);
  traffic_signal.traffic_signal_id = traffic_light_id;

  for (auto & signal : extra_traffic_signals_.signals) {
    if (signal.traffic_signal_id == traffic_light_id) {
      signal = traffic_signal;
      return;
    }
  }

  extra_traffic_signals_.signals.push_back(traffic_signal);
}

void TrafficLightPublishPanel::onResetTrafficLightState()
{
  extra_traffic_signals_.signals.clear();
  enable_publish_ = false;

  publish_button_->setText("PUBLISH");
  publish_button_->setStyleSheet("background-color: #FFFFFF");
}

void TrafficLightPublishPanel::onPublishTrafficLightState()
{
  enable_publish_ = true;

  publish_button_->setText("PUBLISHING...");
  publish_button_->setStyleSheet("background-color: #FFBF00");
}

void TrafficLightPublishPanel::onInitialize()
{
  using std::placeholders::_1;
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  pub_traffic_signals_ = raw_node_->create_publisher<TrafficSignalArray>(
    "/perception/traffic_light_recognition/traffic_signals", 1);
  pub_light_marker_ = raw_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/perception/traffic_light_recognition/traffic_signals/markers", 1);

  sub_vector_map_ = raw_node_->create_subscription<HADMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightPublishPanel::onVectorMap, this, _1));
  sub_traffic_signals_ = raw_node_->create_subscription<TrafficSignalArray>(
    "/perception/traffic_light_recognition/traffic_signals", 1,
    std::bind(&TrafficLightPublishPanel::onTrafficLightSignal, this, _1));
  createWallTimer();

  enable_publish_ = false;
}

void TrafficLightPublishPanel::onRateChanged(int new_rate)
{
  (void)new_rate;
  pub_timer_->cancel();
  createWallTimer();
}

void TrafficLightPublishPanel::createWallTimer()
{
  // convert rate from Hz to milliseconds
  const auto period =
    std::chrono::milliseconds(static_cast<int64_t>(1e3 / publishing_rate_input_->value()));
  pub_timer_ = raw_node_->create_wall_timer(period, [&]() { onTimer(); });
}

void TrafficLightPublishPanel::onTimer()
{
  if (enable_publish_) {
    extra_traffic_signals_.stamp = rclcpp::Clock().now();
    pub_traffic_signals_->publish(extra_traffic_signals_);
  }

  traffic_table_->setRowCount(extra_traffic_signals_.signals.size());

  if (extra_traffic_signals_.signals.empty()) {
    return;
  }

  for (size_t i = 0; i < extra_traffic_signals_.signals.size(); ++i) {
    const auto & signal = extra_traffic_signals_.signals.at(i);

    if (signal.elements.empty()) {
      continue;
    }

    auto id_label = new QLabel(QString::number(signal.traffic_signal_id));
    id_label->setAlignment(Qt::AlignCenter);

    auto color_label = new QLabel();
    color_label->setAlignment(Qt::AlignCenter);

    const auto & light = signal.elements.front();
    switch (light.color) {
      case TrafficSignalElement::RED:
        color_label->setText("RED");
        color_label->setStyleSheet("background-color: #FF0000;");
        break;
      case TrafficSignalElement::AMBER:
        color_label->setText("AMBER");
        color_label->setStyleSheet("background-color: #FFBF00;");
        break;
      case TrafficSignalElement::GREEN:
        color_label->setText("GREEN");
        color_label->setStyleSheet("background-color: #7CFC00;");
        break;
      case TrafficSignalElement::WHITE:
        color_label->setText("WHITE");
        color_label->setStyleSheet("background-color: #FFFFFF;");
        break;
      case TrafficSignalElement::UNKNOWN:
        color_label->setText("UNKNOWN");
        color_label->setStyleSheet("background-color: #808080;");
        break;
      default:
        break;
    }

    auto shape_label = new QLabel();
    shape_label->setAlignment(Qt::AlignCenter);

    switch (light.shape) {
      case TrafficSignalElement::CIRCLE:
        shape_label->setText("CIRCLE");
        break;
      case TrafficSignalElement::LEFT_ARROW:
        shape_label->setText("LEFT_ARROW");
        break;
      case TrafficSignalElement::RIGHT_ARROW:
        shape_label->setText("RIGHT_ARROW");
        break;
      case TrafficSignalElement::UP_ARROW:
        shape_label->setText("UP_ARROW");
        break;
      case TrafficSignalElement::DOWN_ARROW:
        shape_label->setText("DOWN_ARROW");
        break;
      case TrafficSignalElement::DOWN_LEFT_ARROW:
        shape_label->setText("DOWN_LEFT_ARROW");
        break;
      case TrafficSignalElement::DOWN_RIGHT_ARROW:
        shape_label->setText("DOWN_RIGHT_ARROW");
        break;
      case TrafficSignalElement::UNKNOWN:
        shape_label->setText("UNKNOWN");
        break;
      default:
        break;
    }

    auto status_label = new QLabel();
    status_label->setAlignment(Qt::AlignCenter);

    switch (light.status) {
      case TrafficSignalElement::SOLID_OFF:
        status_label->setText("SOLID_OFF");
        break;
      case TrafficSignalElement::SOLID_ON:
        status_label->setText("SOLID_ON");
        break;
      case TrafficSignalElement::FLASHING:
        status_label->setText("FLASHING");
        break;
      case TrafficSignalElement::UNKNOWN:
        status_label->setText("UNKNOWN");
        break;
      default:
        break;
    }

    auto confidence_label = new QLabel(QString::number(light.confidence));
    confidence_label->setAlignment(Qt::AlignCenter);

    traffic_table_->setCellWidget(i, 0, id_label);
    traffic_table_->setCellWidget(i, 1, color_label);
    traffic_table_->setCellWidget(i, 2, shape_label);
    traffic_table_->setCellWidget(i, 3, status_label);
    traffic_table_->setCellWidget(i, 4, confidence_label);
  }
  traffic_table_->update();
}

void TrafficLightPublishPanel::onVectorMap(const HADMapBin::ConstSharedPtr msg)
{
  if (lanelet_map_ptr_) return;
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  const auto tl_reg_elems = lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (auto && tl_reg_elem : tl_reg_elems) {
    auto id = static_cast<int>(tl_reg_elem->id());
    traffic_light_ids_.insert(id);
    tl_reg_elems_map_[id] = tl_reg_elem;
  }

  for (auto && [id, ptr] : tl_reg_elems_map_) {
    traffic_light_id_input_->addItem(QString::fromStdString(std::to_string(id)));
  }
}

static visualization_msgs::msg::Marker lightAsMarker(
  lanelet::ConstPoint3d point, const std::string & color, const std::string ns,
  const rclcpp::Time & current_time)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.frame_locked = true;
  marker.ns = ns;
  marker.id = point.id();
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.pose.position.x = point.x();
  marker.pose.position.y = point.y();
  marker.pose.position.z = point.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  float s = 0.3;

  marker.scale.x = s;
  marker.scale.y = s;
  marker.scale.z = s;

  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.999f;

  if (color.compare("red") == 0) {
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
  } else if (color == "green") {
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
  } else if (color == "yellow") {
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
  } else {
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
  }
  return marker;
}

void TrafficLightPublishPanel::onTrafficLightSignal(const TrafficSignalArray::ConstSharedPtr msg)
{
  visualization_msgs::msg::MarkerArray output_msg;
  const auto current_time = raw_node_->now();

  for (const auto & msg_traffic_signal : msg->signals) {
    const auto msg_traffic_light_it = tl_reg_elems_map_.find(msg_traffic_signal.traffic_signal_id);
    if (msg_traffic_light_it == tl_reg_elems_map_.end()) {
      continue;
    }
    const auto msg_traffic_light = msg_traffic_light_it->second;
    for (const auto & bulbs : msg_traffic_light->lightBulbs()) {
      for (const auto & bulb : bulbs) {
        const std::string color = bulb.attributeOr("color", "none");
        if (color.compare("none") == 0) {
          continue;
        }
        for (const auto & element : msg_traffic_signal.elements) {
          if (color.compare("red") == 0 && element.color == TrafficSignalElement::RED) {
            output_msg.markers.push_back(lightAsMarker(bulb, "red", "traffic_light", current_time));
          } else if (color.compare("green") == 0 && element.color == TrafficSignalElement::GREEN) {
            output_msg.markers.push_back(lightAsMarker(bulb, "red", "traffic_light", current_time));
          } else if (color.compare("yellow") == 0 && element.color == TrafficSignalElement::AMBER) {
            output_msg.markers.push_back(lightAsMarker(bulb, "red", "traffic_light", current_time));
          }
        }
      }
    }
  }
  pub_light_marker_->publish(output_msg);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::TrafficLightPublishPanel, rviz_common::Panel)
