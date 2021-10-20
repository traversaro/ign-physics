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

#ifndef IGNITION_PHYSICS_MUJOCO_PLUGIN_SRC_WORLD_HH_
#define IGNITION_PHYSICS_MUJOCO_PLUGIN_SRC_WORLD_HH_

#include <memory>

#include <ignition/physics/FeatureList.hh>

#include "lib/src/World.hh"

namespace ignition {
namespace physics {
namespace mujocoplugin {

/////////////////////////////////////////////////
class RetrieveWorld : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    /// \brief Get the underlying mujocosim world for this World object.
    public: std::shared_ptr<mujocolib::World> GetmujocoWorld();
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual std::shared_ptr<mujocolib::World> GetmujocoWorld(
        const Identity &_worldID) = 0;
  };
};

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
std::shared_ptr<mujocolib::World> RetrieveWorld::World<PolicyT, FeaturesT>
::GetmujocoWorld()
{
  return this->template Interface<RetrieveWorld>()
      ->GetmujocoWorld(this->identity);
}

}
}
}

#endif
