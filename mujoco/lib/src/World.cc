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

#include <string>
#include <memory>
#include <vector>
#include <filesystem>

#include <tinyxml2.h>

#include <ignition/common/Console.hh>


#include <ignition/common/Profiler.hh>

#include <ignition/math/Pose3.hh>
#include "World.hh"
#include "Model.hh"
#include "Link.hh"

using namespace ignition;
using namespace physics;
using namespace mujocolib;

/////////////////////////////////////////////////
World::World() : Entity()
{
}

/////////////////////////////////////////////////
void World::SetTime(double _time)
{
  this->time = _time;
}

/////////////////////////////////////////////////
double World::GetTime() const
{
  return this->time;
}

/////////////////////////////////////////////////
void World::SetTimeStep(double _timeStep)
{
  this->timeStep = _timeStep;
}

/////////////////////////////////////////////////
double World::GetTimeStep() const
{
  return this->timeStep;
}

/////////////////////////////////////////////////
void World::Step()
{
  IGN_PROFILE("mujocolib::World::Step");
  if (MuJoCoNeedsToBeRegenerated) {
    RegeneratedMuJoCoDataStructures();
    MuJoCoNeedsToBeRegenerated = false;
  }

  // Run MuJoCo integrator
  // TODO implement control of joint torques
  this->mujocoModelPtr->opt.timestep = this->timeStep;
  mj_step(this->mujocoModelPtr, this->mujocoDataPtr);

  // Debug 
  /*
  std::cerr << "time: " << this->time << " mujoco time "  <<  this->mujocoDataPtr->time << "\n";
  for(int i=0; i < this->mujocoModelPtr->nq; i++) {
    std::cerr << "jointPos[" << i << "] = " << this->mujocoDataPtr->qpos[i] << std::endl;
  }

  for(int i=0; i < this->mujocoModelPtr->nbody; i++) {
    std::cerr << "xpos[" << i << "] = " << this->mujocoDataPtr->xpos[i*3+0] << " " 
                                        << this->mujocoDataPtr->xpos[i*3+1] << " " 
                                        << this->mujocoDataPtr->xpos[i*3+2] << std::endl;
    std::cerr << "xquat[" << i << "] = " << this->mujocoDataPtr->xquat[i*4+0] << " " 
                                         << this->mujocoDataPtr->xquat[i*4+1] << " " 
                                         << this->mujocoDataPtr->xquat[i*4+2] << " "
                                         << this->mujocoDataPtr->xquat[i*4+3] << std::endl;
  }*/

  // Propagate changes from MuJoCo's data structures to mujocolib's ones
  // apply updates to each model
  auto &children = this->GetChildren();
  for (auto it = children.begin(); it != children.end(); ++it)
  {
    auto model = std::dynamic_pointer_cast<Model>(it->second);
    model->UpdatePose(this->timeStep);
    auto &ents = model->GetChildren();
    
    for (auto linkIt = ents.begin(); linkIt != ents.end(); ++linkIt)
    {
      // if child of model is link
      auto link = std::dynamic_pointer_cast<Link>(linkIt->second);
      if (link)
      {
        ignition::math::Pose3d world_H_link =
          ignition::math::Pose3d(this->mujocoDataPtr->xpos[link->mujocoId*3+0],
                                 this->mujocoDataPtr->xpos[link->mujocoId*3+1],
                                 this->mujocoDataPtr->xpos[link->mujocoId*3+2],
                                 this->mujocoDataPtr->xquat[link->mujocoId*4+0],
                                 this->mujocoDataPtr->xquat[link->mujocoId*4+1],
                                 this->mujocoDataPtr->xquat[link->mujocoId*4+2],
                                 this->mujocoDataPtr->xquat[link->mujocoId*4+3]);
        link->SetPose(world_H_link);
      }
    }
  }


  // increment world time by step size
  this->time += this->timeStep;
}

/////////////////////////////////////////////////
Entity &World::AddModel()
{
  std::size_t modelId = Entity::GetNextId();
  const auto[it, success] = this->GetChildren().insert(
    {modelId, std::make_shared<Model>(modelId)});
  models.push_back((Model*)it->second.get());
  return *it->second.get();
}

/////////////////////////////////////////////////
std::vector<Contact> World::GetContacts() const
{
  return this->contacts;
}

std::string toString(ignition::math::Vector3d vec)
{
    std::stringstream ss;
    // Related PR: https://github.com/ros/urdfdom_headers/pull/42 .
    ss.imbue(std::locale::classic());
    ss << vec;
    return ss.str();
}

std::string toMuJoCoString(ignition::math::MassMatrix3d massMatrix)
{
    std::stringstream ss;
    // Related PR: https://github.com/ros/urdfdom_headers/pull/42 .
    ss.imbue(std::locale::classic());
    // See https://mujoco.readthedocs.io/en/latest/XMLreference.html#body-inertial
    // for the full inertial serialization
    ss << massMatrix.Ixx() << " " << massMatrix.Iyy() << " " << massMatrix.Izz() << " " 
       << massMatrix.Ixy() << " " << massMatrix.Ixz() << " " << massMatrix.Iyz();
    return ss.str();
}

/////////////////////////////////////////////////
void World::RegeneratedMuJoCoDataStructures()
{
  // First of all, we need to build a list of directed trees out of the joint structure
  // to simplify the mjcf generation. As this is an hack, we have this assumptions:
  // * there is only one model, that is eventually connected to the world
  // * we only have chains, no tree or closed loops
  // * joints and links are passed in the kinematic order 
  // In an actual implementation, this part will need something like Simbody's MultiBodyGraphMaker:
  // https://simtk.org/api_docs/simbody/api_docs32/Simbody/html/classSimTK_1_1MultibodyGraphMaker.html
  struct ChainElement {
    // If ParentLink is nullptr, it is world
    Link* parentLink = nullptr;
    Joint* joint = nullptr ;
    Link* childLink = nullptr;
  };
  std::vector<std::vector<ChainElement>> chains;
  // This simplifies the process 
  chains.resize(this->models.size());
  /*
  Useful for debug: 
  for (auto modelPtr : this->models) {
    std::cerr << "--> Processing model " << modelPtr->GetName() << std::endl;
    for (auto linkPtr : modelPtr->links) {
      std::cerr << "--> Processing link " << linkPtr->GetName() << std::endl;
    }
    for (auto jointPtr : modelPtr->joints) {
      std::cerr << "--> Processing joint " << jointPtr->GetName() << std::endl;
      std::cerr << "----> ( " << jointPtr->parentLinkName << " <--> " << jointPtr->childLinkName << " ) " << std::endl; 
    }
  }*/
  size_t chainId = 0;
  for (auto modelPtr : this->models) {
    chains[chainId].resize(0);
    for (auto jointPtr : modelPtr->joints) {
      ChainElement el;
      if(jointPtr->parentLinkName == "world") { 
        el.parentLink = nullptr;
      } else {
        el.parentLink = modelPtr->GetLinkFromName(jointPtr->parentLinkName);
      }

      el.joint = modelPtr->GetJointFromName(jointPtr->GetName());
      el.childLink = modelPtr->GetLinkFromName(jointPtr->childLinkName);
      chains[chainId].push_back(el);
    }
    chainId++;
  }

  // The generation first generates a MJCF (https://mujoco.readthedocs.io/en/latest/XMLreference.html)
  // using tinyxml2 from the data store in the 
  // ignition::physics::mujocolib::World 
  // and all the related quantities
  tinyxml2::XMLDocument xmlDoc;

  // First tag: mujoco
  tinyxml2::XMLElement* pMujocoEl = xmlDoc.NewElement("mujoco");
  xmlDoc.InsertEndChild(pMujocoEl);

  // Let's be explicit on some options, even if the match the default mjcf settings
  tinyxml2::XMLElement* pCompilerEl = xmlDoc.NewElement("compiler");
  pCompilerEl->SetAttribute("coordinate", "local");
  pCompilerEl->SetAttribute("angle", "radian");
  // Here we do a leap of faith: we trust both SDF and MuJoCo docs that indeed the two
  // euler representation match. A small word of wisdom from a random comment:
  // In general, never trust euler angles documentationuntil you actually check with tests
  // SDF: http://sdformat.org/tutorials?tut=specify_pose
  // MuJoCo: https://mujoco.readthedocs.io/en/latest/XMLreference.html#compiler
  pCompilerEl->SetAttribute("eulerseq", "xyz");
  pCompilerEl->SetAttribute("inertiafromgeom", "false");
  pMujocoEl->InsertEndChild(pCompilerEl);


  // Following tag: worldbody
  tinyxml2::XMLElement* pWorldbodyEl = xmlDoc.NewElement("worldbody");
  pMujocoEl->InsertEndChild(pWorldbodyEl);

  // Add all subsequent links are added following the kinematic structure
  for (auto& chain : chains) {
    tinyxml2::XMLElement* pParentBodyEl = pWorldbodyEl;
    for (auto& chainElement: chain) {
      tinyxml2::XMLElement* pBodyEl = xmlDoc.NewElement("body");
      pBodyEl->SetAttribute("name", chainElement.childLink->GetName().c_str());

      // For what regard the positions of the links, the SDF/World data structure encods them in global coordinaretes 
      // (i.e. world_H_link)
      // (after they have been resolved), while the MJCF default setting (and the most convenient one for several reasons)
      // encodes them as parentLink_H_link
      // so here we just compute
      // parentLink_H_link = compose(inv(world_H_parentLink), world_H_link);
      ignition::math::Pose3d parentLink_H_link;
      // Account for the case in which the parent is indeed the world
      if (chainElement.parentLink == nullptr) {
        parentLink_H_link = chainElement.childLink->GetPose();
      } else {
        //TODO encapsluate as: parentLink_H_link = this->getRelativeTransform(chainElement.parentLink->getName(), chainElement.childLink->getName());
        parentLink_H_link = (chainElement.parentLink->GetPose().Inverse())*chainElement.childLink->GetPose();
      } 
      std::string posStr = toString(parentLink_H_link.Pos());
      std::string rotStr = toString(parentLink_H_link.Rot().Euler());
      pBodyEl->SetAttribute("pos", posStr.c_str());
      pBodyEl->SetAttribute("euler", rotStr.c_str());

      // For FIXED joints, we do not need to add any element, by default the body is fixed
      // to its parent
      // For revolute joints, we add an "hinge" joint
      // example: <joint name="abdomen_z" type="hinge" pos="0 0 0.065" axis="0 0 1" range="-45 45" damping="5" stiffness="20" armature="0.02" />
      //std::cerr << "chainElement.joint->GetName() has type " << chainElement.joint->GetType() << std::endl;

      if (chainElement.joint->GetType() == mujocolib::Joint::Type::Revolute) {
        tinyxml2::XMLElement* pJointEl = xmlDoc.NewElement("joint");
        pJointEl->SetAttribute("name", chainElement.joint->GetName().c_str());
        pJointEl->SetAttribute("type", "hinge");
        std::string axisStr = toString(chainElement.joint->GetAxis());
        pJointEl->SetAttribute("axis", axisStr.c_str());
        pBodyEl->InsertEndChild(pJointEl);
      }

      // Insert inertial data 
      // See https://mujoco.readthedocs.io/en/latest/XMLreference.html#body-inertial
      // <inertial pos="0 0 0" mass="5.20248" fullinertia="0.0964192 0.0964192 0.00936446 0.0 0.0 0.0" />
      tinyxml2::XMLElement* pInertialEl = xmlDoc.NewElement("inertial");
      pInertialEl->SetAttribute("mass", chainElement.childLink->GetInertialParameters().MassMatrix().Mass());
      posStr = toString(chainElement.childLink->GetInertialParameters().Pose().Pos());
      pInertialEl->SetAttribute("pos", posStr.c_str());
      // TODO: rotate full inertia by rotational part so it is expressed with the orientation of the link frame
      // rotStr = toString(chainElement.childLink->GetInertialParameters().Pose().Rot().Euler());
      // pInertialEl->SetAttribute("euler", rotStr.c_str());
      std::string fullinertiaStr = toMuJoCoString(chainElement.childLink->GetInertialParameters().MassMatrix());
      pInertialEl->SetAttribute("fullinertia", fullinertiaStr.c_str());

      pBodyEl->InsertEndChild(pInertialEl);


      pParentBodyEl->InsertEndChild(pBodyEl);
      pParentBodyEl = pBodyEl;
    }
  }
  
  // Print 
  tinyxml2::XMLPrinter printer;
  xmlDoc.Print(&printer);

  std::cerr << "MuJoCo model (MJCF) generated on the fly from Ignition Physics API calls: " << std::endl << printer.CStr() << std::endl;

  // Cleanup previous model if any
  if (this->mujocoModelPtr) {
    mj_deleteModel(this->mujocoModelPtr);
    this->mujocoModelPtr = nullptr;
  }

  if (this->mujocoDataPtr) {
    mj_deleteData(this->mujocoDataPtr);
    this->mujocoDataPtr = nullptr;
  }

  // Write generated MJCF to a temporary file
  // This should have some kind of unique name
  auto tmpFileName = std::filesystem::temp_directory_path() / "mujoco_ignition_temp_file.mjcf";
  std::ofstream out(tmpFileName.c_str());
  out << printer.CStr();
  out.close();

  // Regenerate mujoco model and data 
  char mujoco_error[2000];
  this->mujocoModelPtr = mj_loadXML(tmpFileName.c_str(), NULL, mujoco_error, 2000);

  if (this->mujocoModelPtr == NULL) { 
    ignerr << "Error when compiling mjcf file " << std::endl;
    ignerr << mujoco_error << std::endl;
    return;
  }

  // Options can always be set by the user at runtime, so there is no need to serialize them in the xml
  this->mujocoModelPtr->opt.timestep = this->timeStep;
  
  // TODO handle errors 
  this->mujocoDataPtr = mj_makeData(this->mujocoModelPtr);

  // Let's update the mujodo id in the mujocolib data structures
  for (auto modelPtr : this->models) {
    for (auto linkPtr : modelPtr->links) {
      for(int i=0; i < this->mujocoModelPtr->nbody; i++) {
        std::string bodyName(this->mujocoModelPtr->names+this->mujocoModelPtr->name_bodyadr[i]);
        if (bodyName == linkPtr->GetName()) {
          linkPtr->mujocoId = i;
        }
      }
    }
  }
}
