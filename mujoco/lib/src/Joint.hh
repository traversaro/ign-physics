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

#ifndef IGNITION_PHYSICS_MUJOCO_LIB_SRC_JOINT_HH_
#define IGNITION_PHYSICS_MUJOCO_LIB_SRC_JOINT_HH_

#include <ignition/utilities/SuppressWarning.hh>

#include "ignition/physics/mujocolib/Export.hh"

#include "Entity.hh"
#include "Link.hh"

#include <ignition/math/Inertial.hh>

namespace ignition {
namespace physics {
namespace mujocolib {

/// \brief Joint class
class IGNITION_PHYSICS_MUJOCOLIB_VISIBLE Joint : public Entity
{ 
  public: enum Type {
    Revolute,
    Fixed
  };

  /// \brief Constructor
  public: Joint();

  /// \brief Constructor
  /// \param[in] _id Joint id
  public: explicit Joint(std::size_t _id);

  /// \brief Destructor
  public: virtual ~Joint() = default;

  public: void SetAxis(const math::Vector3d &_axis);

  public: math::Vector3d GetAxis() const;

  public: void SetType(const Type &_type);

  public: Type GetType() const;

  IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  protected: math::Vector3d axis;

  protected: Type type;

  public: std::string parentLinkName;
  public: std::string childLinkName;

  IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
};

}
}
}

#endif
