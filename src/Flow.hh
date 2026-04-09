/*
 * Copyright (C) 2026 Olivier Kermorgant
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

#ifndef BOATING_FLOW_HH_
#define BOATING_FLOW_HH_

#include <gz/sim/System.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/sim/components.hh>

namespace boating
{
  /// \brief A simple class to subscribe to air or water current and provide it to other plugins

class Flow
{

private:
  std::mutex mtx;
  gz::math::Vector3d currentVector{0, 0, 0};
  gz::transport::Node node;

  inline void onCurrentMsg(const gz::msgs::Vector3d &_msg)
  {
    std::lock_guard lock(mtx);
    currentVector = gz::msgs::Convert(_msg);
  }

public:

  inline void subcribeToCurrent(const std::string &topic)
  {
    node.Subscribe(topic, &Flow::onCurrentMsg, this);
  }

  // get the current flow in world frame
  inline auto flow() const
  {
    std::lock_guard lock(mtx);
    return currentVector;
  }
  };
}

#endif
