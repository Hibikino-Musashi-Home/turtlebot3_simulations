// Copyright 2012 Open Source Robotics Foundation
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
// Author: Ryan Shim, ChanHyeong Lee

#ifndef TURTLEBOT3_GAZEBO__OBSTACLE1_HPP_
#define TURTLEBOT3_GAZEBO__OBSTACLE1_HPP_

#include <chrono>
#include <memory>
#include <vector>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/math/Vector3.hh>

namespace turtlebot3_gazebo
{

class Obstacle1Plugin
  : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
public:
  Obstacle1Plugin() = default;
  ~Obstacle1Plugin() override = default;

  void Configure(
    const ignition::gazebo::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    ignition::gazebo::EntityComponentManager & ecm,
    ignition::gazebo::EventManager & eventMgr) override;

  void PreUpdate(
    const ignition::gazebo::UpdateInfo & info,
    ignition::gazebo::EntityComponentManager & ecm) override;

private:
  ignition::gazebo::Model model;
  std::chrono::steady_clock::time_point startTime;

  std::vector<gz::math::Vector3d> waypoints;
  std::vector<double> segmentDistances;
  double totalDistance = 0.0;
  double speed = 0.1;  // meters per second
};

}  // namespace turtlebot3_gazebo

#endif  // TURTLEBOT3_GAZEBO__OBSTACLE1_HPP_
