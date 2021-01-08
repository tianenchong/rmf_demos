/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <bits/stdc++.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <google/protobuf/descriptor.h>
#include <iostream>
#include <gazebo_ros/node.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_mode.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>
#include <rmf_door_msgs/msg/supervisor_heartbeat.hpp>

namespace gazebo {
using namespace google::protobuf;
class DoorWatcherNode;
typedef std::shared_ptr<DoorWatcherNode> _DWN_PTR;
_DWN_PTR dwn_ptr = nullptr;
class DoorWatcherNode : public rclcpp::Node
{
public:
  const std::string FinalDoorRequestTopicName = "door_requests";
  const std::string AdapterDoorRequestTopicName = "adapter_door_requests";
  const std::string DoorStateTopicName = "door_states";
  const std::string DoorSupervisorHeartbeatTopicName =
    "door_supervisor_heartbeat";
  using DoorMode = rmf_door_msgs::msg::DoorMode;
  using DoorRequest = rmf_door_msgs::msg::DoorRequest;
  using DoorRequestPub = rclcpp::Publisher<DoorRequest>;
  std::vector<double> _previous_scan;
  DoorRequestPub::SharedPtr _door_request_pub;
  DoorWatcherNode()
  : rclcpp::Node("door_watcher")
  {
    const auto default_qos = rclcpp::SystemDefaultsQoS();
    _door_request_pub = create_publisher<DoorRequest>(
      FinalDoorRequestTopicName, default_qos);
    DoorRequest request;
    request.door_name = "coe_door";
    request.request_time = get_clock()->now();
    request.requester_id = "door_watcher";
    request.requested_mode.value = DoorMode::MODE_OPEN;
    _door_request_pub->publish(request);
  }
  ~DoorWatcherNode() {}
  static void cb(ConstLaserScanStampedPtr& _msg)
  {
    // Dump the message contents to stdout.
    //const std::string data;
    if (_msg->has_scan())
    {
      //std::cout << _msg->DebugString();
      const auto scan = _msg->scan();
      const auto* descriptor = scan.GetDescriptor();
      const auto* ranges_field = descriptor->FindFieldByName("ranges");
      assert(ranges_field != nullptr);
      assert(ranges_field->type() == FieldDescriptor::TYPE_DOUBLE);
      assert(ranges_field->label() == FieldDescriptor::LABEL_REPEATED);
      const auto* reflection = scan.GetReflection();
      auto count = scan.count();
      const auto* scan_msg =
        dynamic_cast<const Message*>(&scan);

      std::vector<double> current_scan = {};
      for (unsigned int i = 0; i < count; i++)
      {
        current_scan.push_back(std::move(reflection->GetRepeatedDouble(*scan_msg,
          ranges_field, i)));
      }


      if (dwn_ptr->_previous_scan.size() != 0)
      {
        DoorRequest request;
        request.door_name = "coe_door";
        request.request_time = dwn_ptr->get_clock()->now();
        request.requester_id = "door_watcher";
        bool same = true;
        for (unsigned int i = 0; i < count; i++)
        {
          if (trunc(current_scan[i]*1000) !=
            trunc((dwn_ptr->_previous_scan)[i]*1000))
          {
            same = false;
            //std::cout << "["<< i <<"] "<< current_scan[i] << " " <<
            //(dwn_ptr->_previous_scan)[i] << std::endl;
            break;
          }
        }
        request.requested_mode.value =
          (same) ? DoorMode::MODE_CLOSED : DoorMode::
          MODE_OPEN;
        dwn_ptr->_door_request_pub->publish(request);
      }
      dwn_ptr->_previous_scan = current_scan;
    }

  }
  static void listener_main()
  {
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo world_stats topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe(
      "~/coe_door/coe_door_sensor_2/link/hokuyo/scan", DoorWatcherNode::cb);

    while (true)
      gazebo::common::Time::MSleep(10);
  }

};

class DoorWatcher : public WorldPlugin
{
public: DoorWatcher()
  : WorldPlugin()
  {
    printf("Door Watcher created.\n");
    dwn_ptr = std::make_shared<DoorWatcherNode>();

    _main_thread = std::make_unique<std::thread>(
      &DoorWatcherNode::listener_main);

    _main_thread->detach();
  }

public: ~DoorWatcher()
  {
    if (_main_thread->joinable())
      _main_thread->join();
  }

public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {}

public: void Init() {}

public: virtual void Reset() {}
private:
  std::unique_ptr<std::thread> _main_thread;
};
GZ_REGISTER_WORLD_PLUGIN(DoorWatcher)
}
