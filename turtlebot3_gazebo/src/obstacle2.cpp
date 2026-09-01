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

#include "turtlebot3_gazebo/obstacle2.hpp"

#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Element.hh>

namespace turtlebot3_gazebo
{

void Obstacle2Plugin::Configure(
  const ignition::gazebo::Entity & entity,
  const std::shared_ptr<const sdf::Element> &,
  ignition::gazebo::EntityComponentManager &,
  ignition::gazebo::EventManager &)
{
  this->model = ignition::gazebo::Model(entity);
  this->startTime = std::chrono::steady_clock::now();

  this->waypoints = {
    {-2.0, -2.0, 0.25},
    {-1.3, -1.8, 0.25},
    {0.5, 2.0, 0.25},
    {-2.0, 1.5, 0.25},
    {1.5, -0.2, 0.25},
    {1.5, -2.0, 0.25},
    {0.0, -1.5, 0.25},
    {-0.5, -1.0, 0.25},
    {-1.0, -1.5, 0.25},
    {-1.5, -1.9, 0.25},
    {-2.0, -2.0, 0.25}
  };

  this->segmentDistances.clear();
  this->totalDistance = 0.0;

  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    double dist = (waypoints[i + 1] - waypoints[i]).Length();
    this->segmentDistances.push_back(dist);
    this->totalDistance += dist;
  }
}

void Obstacle2Plugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &,
  ignition::gazebo::EntityComponentManager & ecm)
{
  if (!this->model.Valid(ecm)) {return;}

  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = now - this->startTime;

  double travelDist = std::fmod(elapsed.count() * this->speed, this->totalDistance);

  size_t idx = 0;
  double acc = 0.0;

  while (idx < segmentDistances.size() && acc + segmentDistances[idx] < travelDist) {
    acc += segmentDistances[idx];
    ++idx;
  }

  if (idx >= segmentDistances.size()) {return;}

  double localT = (travelDist - acc) / segmentDistances[idx];
  ignition::math::Vector3d start = waypoints[idx];
  ignition::math::Vector3d end = waypoints[idx + 1];
  ignition::math::Vector3d currentPos = start + (end - start) * localT;

  ignition::math::Pose3d pose(currentPos, ignition::math::Quaterniond::Identity);
  this->model.SetWorldPoseCmd(ecm, pose);
}

}  // namespace turtlebot3_gazebo

IGNITION_ADD_PLUGIN(
  turtlebot3_gazebo::Obstacle2Plugin,
  ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  turtlebot3_gazebo::Obstacle2Plugin,
  "turtlebot3_gazebo::Obstacle2Plugin")
