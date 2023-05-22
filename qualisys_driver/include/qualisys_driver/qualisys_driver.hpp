// Copyright 2020 National Institute of Advanced Industrial Science and Technology, Japan
// Copyright 2019 Intelligent Robotics Lab
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
//
// Author: Floris Erich <floris.erich@aist.go.jp>
//         David Vargas Frutos <david.vargas@urjc.es>
//
// Also includes code fragments from Kumar Robotics ROS 1 Qualisys driver

#ifndef QUALISYS_DRIVER__QUALISYS_DRIVER_HPP_
#define QUALISYS_DRIVER__QUALISYS_DRIVER_HPP_

#include <iostream>
#include <sstream>
#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/node_interfaces/node_logging.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "mocap_msgs/msg/marker.hpp"
#include "mocap_msgs/msg/markers.hpp"
#include "mocap_msgs/msg/rigid_body.hpp"
#include "mocap_msgs/msg/rigid_bodies.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2/buffer_core.h"
#include "tf2_ros/transform_broadcaster.h"

#include "RTProtocol.h"

class QualisysDriver : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit QualisysDriver(
    const rclcpp::NodeOptions options =
    rclcpp::NodeOptions().parameter_overrides(
      std::vector<rclcpp::Parameter> {
    rclcpp::Parameter("use_sim_time", true)
  }
    )
  );

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);
  bool connect_qualisys();
  void set_settings_qualisys();
  void loop();
  bool stop_qualisys();
  void initParameters();

private:
  std::shared_ptr<rclcpp::TimerBase> timer_;

  void process_packet(CRTPacket * const);

  CRTProtocol port_protocol_;
  std::string host_name_;
  int port_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string qos_history_policy_;
  std::string qos_reliability_policy_;
  int qos_depth_;
  bool use_markers_with_id_;
  int last_frame_number_;
  int frame_count_;
  int dropped_frame_count_;
  int n_markers_;
  int n_unlabeled_markers_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  rclcpp_lifecycle::LifecyclePublisher<mocap_msgs::msg::Markers>::SharedPtr mocap_markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mocap_msgs::msg::RigidBodies>::SharedPtr
    mocap_rigid_bodies_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr update_pub_;
};

static
std::map<std::string, rmw_qos_reliability_policy_t> name_to_reliability_policy_map = {
  {"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
  {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT}
};

static
std::map<std::string, rmw_qos_history_policy_t> name_to_history_policy_map = {
  {"keep_last", RMW_QOS_POLICY_HISTORY_KEEP_LAST},
  {"keep_all", RMW_QOS_POLICY_HISTORY_KEEP_ALL}
};

#endif  // QUALISYS_DRIVER__QUALISYS_DRIVER_HPP_
