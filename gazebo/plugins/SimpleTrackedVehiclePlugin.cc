/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <functional>
#include <vector>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

// OpenDE private definitions; unfortunately, we need them
#include "joints/contact.h"

#include "gazebo/common/Assert.hh"
#include "gazebo/transport/transport.hh"

#include "plugins/SimpleTrackedVehiclePlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SimpleTrackedVehiclePlugin)

void SimpleTrackedVehiclePlugin::Load(physics::ModelPtr _model,
                                      sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "SimpleTrackedVehiclePlugin: _model pointer is NULL");
  GZ_ASSERT(_sdf, "SimpleTrackedVehiclePlugin: _sdf pointer is NULL");

  if (_model->GetWorld()->Physics()->GetType() != "ode")
  {
    gzerr << "Tracked vehicle simulation works only with ODE." << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  TrackedVehiclePlugin::Load(_model, _sdf);

  if (!_sdf->HasElement("body"))
  {
    gzerr << "SimpleTrackedVehiclePlugin: <body> tag missing." << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  if (!_sdf->HasElement("left_track"))
  {
    gzerr << "SimpleTrackedVehiclePlugin: <left_track> tag missing."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  if (!_sdf->HasElement("right_track"))
  {
    gzerr << "SimpleTrackedVehiclePlugin: <right_track> tag missing."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  if (!_sdf->HasElement("rear_left_track"))
  {
    gzerr << "SimpleTrackedVehiclePlugin: <rear_left_track> tag missing."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  if (!_sdf->HasElement("rear_right_track"))
  {
    gzerr << "SimpleTrackedVehiclePlugin: <rear_right_track> tag missing."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  if (!_sdf->HasElement("front_left_track"))
  {
    gzerr << "SimpleTrackedVehiclePlugin: <front_left_track> tag missing."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  if (!_sdf->HasElement("front_right_track"))
  {
    gzerr << "SimpleTrackedVehiclePlugin: <front_right_track> tag missing."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  this->body = _model->GetLink(
      _sdf->GetElement("body")->Get<std::string>());
  if (this->body == nullptr)
  {
    gzerr << "SimpleTrackedVehiclePlugin: <body> link does not exist."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  this->tracks[Tracks::LEFT] = _model->GetLink(
      _sdf->GetElement("left_track")->Get<std::string>());

  gzdbg<<(_sdf->GetElement("body")->Get<std::string>())<<std::endl;
  if (_model->GetLink("left_track") == nullptr)
  {
    gzerr << "EEEEY"
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }
  if (this->tracks[Tracks::LEFT] == nullptr)
  {
    gzerr << "SimpleTrackedVehiclePlugin: <left_track> link does not exist."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  this->tracks[Tracks::RIGHT] = _model->GetLink(
      _sdf->GetElement("right_track")->Get<std::string>());
  if (this->tracks[Tracks::RIGHT] == nullptr)
  {
    gzerr << "SimpleTrackedVehiclePlugin: <right_track> link does not exist."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  this->tracks[Tracks::REAR_LEFT] = _model->GetLink(
      _sdf->GetElement("rear_left_track")->Get<std::string>());
  if (this->tracks[Tracks::RIGHT] == nullptr)
  {
    gzerr << "SimpleTrackedVehiclePlugin: <rear_left_track> link does not exist."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  this->tracks[Tracks::REAR_RIGHT] = _model->GetLink(
      _sdf->GetElement("rear_right_track")->Get<std::string>());
  if (this->tracks[Tracks::RIGHT] == nullptr)
  {
    gzerr << "SimpleTrackedVehiclePlugin: <rear_right_track> link does not exist."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  this->tracks[Tracks::FRONT_LEFT] = _model->GetLink(
      _sdf->GetElement("front_left_track")->Get<std::string>());
  if (this->tracks[Tracks::RIGHT] == nullptr)
  {
    gzerr << "SimpleTrackedVehiclePlugin: <front_left_track> link does not exist."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  this->tracks[Tracks::FRONT_RIGHT] = _model->GetLink(
      _sdf->GetElement("front_right_track")->Get<std::string>());
  if (this->tracks[Tracks::RIGHT] == nullptr)
  {
    gzerr << "SimpleTrackedVehiclePlugin: <front_right_track> link does not exist."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  this->LoadParam(_sdf, "collide_without_contact_bitmask",
                  this->collideWithoutContactBitmask, 1u);
}

void SimpleTrackedVehiclePlugin::Init()
{
  TrackedVehiclePlugin::Init();

  physics::ModelPtr model = this->body->GetModel();

  this->contactManager = model->GetWorld()->Physics()->GetContactManager();
  // otherwise contact manager would not publish any contacts (since we are not
  // a real contact subscriber)
  this->contactManager->SetNeverDropContacts(true);

  // set correct categories and collide bitmasks
  this->SetGeomCategories();

  // set the desired friction to tracks (override the values set in the
  // SDF model)
  this->UpdateTrackSurface();

  // initialize Gazebo node, subscribers and publishers and event connections
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(model->GetWorld()->Name());

  this->beforePhysicsUpdateConnection =
      event::Events::ConnectBeforePhysicsUpdate(
          std::bind(&SimpleTrackedVehiclePlugin::DriveTracks, this,
                    std::placeholders::_1));
}

void SimpleTrackedVehiclePlugin::Reset()
{
  TrackedVehiclePlugin::Reset();
}

void SimpleTrackedVehiclePlugin::SetTrackVelocityImpl(double _left,
                                                      double _right,
                                                      double _rear_left,
                                                      double _rear_right,
                                                      double _front_left,
                                                      double _front_right)
{
  this->trackVelocity[Tracks::LEFT] = _left;
  this->trackVelocity[Tracks::RIGHT] = _right;
  this->trackVelocity[Tracks::REAR_LEFT] = _rear_left;
  this->trackVelocity[Tracks::REAR_RIGHT] = _rear_right;
  this->trackVelocity[Tracks::FRONT_LEFT] = _front_left;
  this->trackVelocity[Tracks::FRONT_RIGHT] = _front_right;
}

void SimpleTrackedVehiclePlugin::UpdateTrackSurface()
{
  for (auto track : this->tracks )
  {
    this->SetLinkMu(track.second);
  }
}

void SimpleTrackedVehiclePlugin::SetGeomCategories()
{
  auto linksToProcess = this->body->GetModel()->GetLinks();

  // set ROBOT_CATEGORY to the whole body and all subparts
  physics::LinkPtr link;
  while (!linksToProcess.empty())
  {
    link = linksToProcess.back();
    linksToProcess.pop_back();

    auto childLinks = link->GetChildJointsLinks();
    linksToProcess.insert(linksToProcess.end(), childLinks.begin(),
                          childLinks.end());

    for (auto const &collision : link->GetCollisions())
    {
      collision->SetCategoryBits(ROBOT_CATEGORY);
      collision->SetCollideBits(GZ_FIXED_COLLIDE);

      GZ_ASSERT(collision->GetSurface() != nullptr,
                "Collision surface is nullptr");
      collision->GetSurface()->collideWithoutContactBitmask =
        this->collideWithoutContactBitmask;
    }
  }

  for (auto track : this->tracks)
  {
    auto trackLink = track.second;
    for (auto const &collision : trackLink->GetCollisions())
    {
      auto bits = ROBOT_CATEGORY | BELT_CATEGORY;
      if (track.first == Tracks::LEFT ||
          track.first == Tracks::REAR_LEFT ||
          track.first == Tracks::FRONT_LEFT)
        bits |= LEFT_CATEGORY;

      collision->SetCategoryBits(bits);
    }
  }
}

void SimpleTrackedVehiclePlugin::DriveTracks(
    const common::UpdateInfo &/*_unused*/)
{
  if (this->contactManager->GetContactCount() == 0)
    return;
  /////////////////////////////////////////////
  // Calculate the desired center of rotation
  /////////////////////////////////////////////

  const auto leftBeltSpeed = -this->trackVelocity[Tracks::LEFT];
  const auto rightBeltSpeed = -this->trackVelocity[Tracks::RIGHT];
  const auto rearleftBeltSpeed = -this->trackVelocity[Tracks::REAR_LEFT];
  const auto rearrightBeltSpeed = -this->trackVelocity[Tracks::REAR_RIGHT];
  const auto frontleftBeltSpeed = -this->trackVelocity[Tracks::FRONT_LEFT];
  const auto frontrightBeltSpeed = -this->trackVelocity[Tracks::FRONT_RIGHT];

  // the desired linear and angular speeds (set by desired track velocities)
  const auto linearSpeed = (leftBeltSpeed + 
                            rightBeltSpeed + 
                            rearleftBeltSpeed +
                            rearrightBeltSpeed + 
                            frontleftBeltSpeed + 
                            frontrightBeltSpeed) / 6;
  const auto angularSpeed = -(((leftBeltSpeed + rearleftBeltSpeed + frontleftBeltSpeed)/3)
                            -((rightBeltSpeed + rearrightBeltSpeed + frontrightBeltSpeed)/3)) *
    this->GetSteeringEfficiency() / this->GetTracksSeparation();

  // radius of the turn the robot is doing
  const auto desiredRotationRadiusSigned =
                               (fabs(angularSpeed) < 0.1) ?
                               // is driving straight
                               dInfinity :
                               (
                                 (fabs(linearSpeed) < 0.1) ?
                                 // is rotating about a single point
                                 0 :
                                 // general movement
                                 linearSpeed / angularSpeed);

  const auto bodyPose = this->body->WorldPose();
  const auto bodyYAxisGlobal =
    bodyPose.Rot().RotateVector(ignition::math::Vector3d(0, 1, 0));
  const auto centerOfRotation =
    (bodyYAxisGlobal * desiredRotationRadiusSigned) + bodyPose.Pos();

  ////////////////////////////////////////////////////////////////////////
  // For each contact, compute the friction force direction and speed of
  // surface movement.
  ////////////////////////////////////////////////////////////////////////
  size_t i = 0;
  const auto contacts = this->contactManager->GetContacts();

  for (auto contact : contacts)
  {
    // Beware! There may be invalid contacts beyond GetContactCount()...
    if (i == this->contactManager->GetContactCount())
      break;

    ++i;

    if (contact->collision1->GetSurface()->collideWithoutContact ||
      contact->collision2->GetSurface()->collideWithoutContact)
      continue;

    if (!contact->collision1->GetLink()->GetEnabled() ||
      !contact->collision2->GetLink()->GetEnabled())
      continue;

    if (contact->collision1->IsStatic() && contact->collision2->IsStatic())
    {
      // we're not interested in static model collisions
      // (they do not have any ODE bodies).
      continue;
    }

    dBodyID body1 = dynamic_cast<physics::ODELink&>(
      *contact->collision1->GetLink()).GetODEId();
    dBodyID body2 = dynamic_cast<physics::ODELink& >(
      *contact->collision2->GetLink()).GetODEId();
    dGeomID geom1 = dynamic_cast<physics::ODECollision& >(
      *contact->collision1).GetCollisionId();
    dGeomID geom2 = dynamic_cast<physics::ODECollision& >(
      *contact->collision2).GetCollisionId();

    bool bodiesSwapped = false;
    if (body1 == 0)
    {
      std::swap(body1, body2);
      std::swap(geom1, geom2);

      // we'll take care of the normal flipping later
      bodiesSwapped = true;
    }

    // determine if track is the first or second collision element
    const bool isGeom1Track = (dGeomGetCategoryBits(geom1) & BELT_CATEGORY) > 0;
    const bool isGeom2Track = (dGeomGetCategoryBits(geom2) & BELT_CATEGORY) > 0;

    if (!isGeom1Track && !isGeom2Track)
      continue;

    // speed and geometry of the track in collision
    const auto trackGeom = (isGeom1Track ? geom1 : geom2);
    // the != means XOR here; we basically want to get the collision belonging
    // to the track, but we might have swapped the ODE bodies in between,
    // so we have to account for it
    const physics::Collision* trackCollision =
      ((isGeom1Track != bodiesSwapped) ? contact->collision1
                                       : contact->collision2);
    const dReal beltSpeed =
      (dGeomGetCategoryBits(trackGeom) & LEFT_CATEGORY) != 0 ?
      leftBeltSpeed : rightBeltSpeed;

    // remember if we've found at least one contact joint (we should!)
    bool foundContact = false;
    for (auto contactIterator = ContactIterator::begin(body1, geom1, geom2);
         contactIterator != ContactIterator::end();
         ++contactIterator)
    {
      dContact* odeContact = contactIterator.getPointer();

      // now we're sure it is a contact between our two geometries
      foundContact = true;

      const ignition::math::Vector3d contactWorldPosition(
        odeContact->geom.pos[0],
        odeContact->geom.pos[1],
        odeContact->geom.pos[2]);

      ignition::math::Vector3d contactNormal(
        odeContact->geom.normal[0],
        odeContact->geom.normal[1],
        odeContact->geom.normal[2]);

      // We always want contactNormal to point "inside" the track.
      // The dot product is 1 for co-directional vectors and -1 for
      // opposite-pointing vectors.
      // The contact can be flipped either by swapping body1 and body2 above,
      // or by having some flipped faces on collision meshes.
      const double normalToTrackCenterDot =
        contactNormal.Dot(
          trackCollision->WorldPose().Pos() - contactWorldPosition);
      if (normalToTrackCenterDot < 0)
      {
        contactNormal = -contactNormal;
      }

      // vector tangent to the belt pointing in the belt's movement direction
      auto beltDirection(contactNormal.Cross(bodyYAxisGlobal));

      if (beltSpeed > 0)
        beltDirection = -beltDirection;

      const auto frictionDirection =
        this->ComputeFrictionDirection(linearSpeed,
                                       angularSpeed,
                                       desiredRotationRadiusSigned == dInfinity,
                                       bodyPose,
                                       bodyYAxisGlobal,
                                       centerOfRotation,
                                       contactWorldPosition,
                                       contactNormal,
                                       beltDirection);

      odeContact->fdir1[0] = frictionDirection.X();
      odeContact->fdir1[1] = frictionDirection.Y();
      odeContact->fdir1[2] = frictionDirection.Z();

      // use friction direction and motion1 to simulate the track movement
      odeContact->surface.mode |= dContactFDir1 | dContactMotion1;

      odeContact->surface.motion1 = this->ComputeSurfaceMotion(
        beltSpeed, beltDirection, frictionDirection);
    }

    if (!foundContact)
    {
      gzwarn << "No ODE contact joint found for contact " <<
             contact->DebugString() << std::endl;
      continue;
    }
  }
}

ignition::math::Vector3d SimpleTrackedVehiclePlugin::ComputeFrictionDirection(
  const double _linearSpeed, const double _angularSpeed,
  const bool _drivingStraight, const ignition::math::Pose3d &_bodyPose,
  const ignition::math::Vector3d &_bodyYAxisGlobal,
  const ignition::math::Vector3d &_centerOfRotation,
  const ignition::math::Vector3d &_contactWorldPosition,
  const ignition::math::Vector3d &_contactNormal,
  const ignition::math::Vector3d &_beltDirection) const
{
  ignition::math::Vector3d frictionDirection;

  if (!_drivingStraight)
  {
    // non-straight drive

    // vector pointing from the center of rotation to the contact point
    const auto COR2Contact =
      (_contactWorldPosition - _centerOfRotation).Normalize();

    // the friction force should be perpendicular to COR2Contact
    frictionDirection = _contactNormal.Cross(COR2Contact);

    // position of the contact point relative to vehicle body
    const auto contactInVehiclePos =
        _bodyPose.Rot().RotateVectorReverse(
          _contactWorldPosition - _bodyPose.Pos());

    const int linearSpeedSignum =
        (fabs(_linearSpeed) > 0.1) ? ignition::math::signum(_linearSpeed) : 1;

    // contactInVehiclePos.Dot(ignition::math::Vector3d(1, 0, 0)) > 0 means
    // the contact is "in front" of the line on which COR moves
    if ((ignition::math::signum(_angularSpeed) *
      ignition::math::signum(_bodyYAxisGlobal.Dot(frictionDirection))) !=
      (linearSpeedSignum *
        ignition::math::signum(contactInVehiclePos.Dot(
          ignition::math::Vector3d(1, 0, 0)))))
    {
      frictionDirection = -frictionDirection;
    }

    if (_linearSpeed < 0)
      frictionDirection = - frictionDirection;
  }
  else
  {
    // straight drive
    frictionDirection = _contactNormal.Cross(_bodyYAxisGlobal);

    if (frictionDirection.Dot(_beltDirection) < 0)
      frictionDirection = -frictionDirection;
  }

  return frictionDirection;
}

double SimpleTrackedVehiclePlugin::ComputeSurfaceMotion(const double _beltSpeed,
    const ignition::math::Vector3d &_beltDirection,
    const ignition::math::Vector3d &_frictionDirection) const
{
  // the dot product <beltDirection,fdir1> is the cosine of the angle they
  // form (because both are unit vectors)
  // the motion is in the opposite direction than the desired motion of the body
  return -_beltDirection.Dot(_frictionDirection) * fabs(_beltSpeed);
}

SimpleTrackedVehiclePlugin::ContactIterator
SimpleTrackedVehiclePlugin::ContactIterator::operator++()
{
  // initialized && null contact means we've reached the end of the iterator
  if (this->initialized && this->currentContact == nullptr)
  {
    return *this;
  }

  // I haven't found a nice way to get ODE ID of the collision joint,
  // so we need to iterate over all joints connecting the two colliding
  // bodies and try to find the one we're interested in.
  // This should not be a performance issue, since bodies connected by other
  // joint types do not collide by default.

  // remember if we've found at least one contact joint (we should!)
  bool found = false;
  for (; this->jointIndex < static_cast<size_t>(dBodyGetNumJoints(this->body));
         this->jointIndex++)
  {
    const auto joint = dBodyGetJoint(this->body,
                                     static_cast<int>(this->jointIndex));

    // only interested in contact joints
    if (dJointGetType(joint) != dJointTypeContact)
      continue;

    // HACK here we unfortunately have to access private ODE data
    // It must really be static_cast here; if dynamic_cast is used, the runtime
    // cannot find RTTI for dxJointContact and its predecessors.
    dContact* odeContact = &(static_cast<dxJointContact*>(joint)->contact);

    if (!(
            odeContact->geom.g1 == this->geom1 &&
            odeContact->geom.g2 == this->geom2)
        &&
        !(
            odeContact->geom.g1 == this->geom2 &&
            odeContact->geom.g2 == this->geom1))
    {
      // not a contact between our two geometries
      continue;
    }

    // we found a contact we're interested in

    found = true;
    this->initialized = true;

    // we can be pretty sure the contact instance won't get deleted until this
    // code finishes, since we are in a pause between contact generation and
    // physics update
    this->currentContact = odeContact;

    // needed since we break out of the for-loop
    this->jointIndex++;
    break;
  }

  if (!found)
  {
    // we've reached the end of the iterator
    this->currentContact = nullptr;
    this->initialized = true;
  }

  this->initialized = true;
  return *this;
}

SimpleTrackedVehiclePlugin::ContactIterator::ContactIterator()
    : currentContact(nullptr), jointIndex(0), body(nullptr), geom1(nullptr),
      geom2(nullptr), initialized(false)
{
}

SimpleTrackedVehiclePlugin::ContactIterator::ContactIterator(
    bool _initialized) : currentContact(nullptr), jointIndex(0), body(nullptr),
                         geom1(nullptr), geom2(nullptr),
                         initialized(_initialized)
{
}

SimpleTrackedVehiclePlugin::ContactIterator::ContactIterator(
    const SimpleTrackedVehiclePlugin::ContactIterator &_rhs)
{
  this->currentContact = _rhs.currentContact;
  this->initialized = _rhs.initialized;
  this->jointIndex = _rhs.jointIndex;
  this->body = _rhs.body;
  this->geom1 = _rhs.geom1;
  this->geom2 = _rhs.geom2;
}

SimpleTrackedVehiclePlugin::ContactIterator::ContactIterator(
    dBodyID _body, dGeomID _geom1, dGeomID _geom2) :
    currentContact(nullptr), jointIndex(0), body(_body),
    geom1(_geom1), geom2(_geom2), initialized(false)
{
}

SimpleTrackedVehiclePlugin::ContactIterator
SimpleTrackedVehiclePlugin::ContactIterator::begin(
    dBodyID _body, dGeomID _geom1, dGeomID _geom2)
{
  return ContactIterator(_body, _geom1, _geom2);
}

SimpleTrackedVehiclePlugin::ContactIterator
SimpleTrackedVehiclePlugin::ContactIterator::end()
{
  return ContactIterator(true);
}

bool SimpleTrackedVehiclePlugin::ContactIterator::operator==(
    const SimpleTrackedVehiclePlugin::ContactIterator &_rhs)
{
  if (this->currentContact == nullptr && !this->initialized)
    ++(*this);

  return this->currentContact == _rhs.currentContact &&
         this->initialized == _rhs.initialized;
}

SimpleTrackedVehiclePlugin::ContactIterator&
SimpleTrackedVehiclePlugin::ContactIterator::operator=(
    const SimpleTrackedVehiclePlugin::ContactIterator &_rhs)
{
  this->currentContact = _rhs.currentContact;
  this->initialized = _rhs.initialized;
  this->jointIndex = _rhs.jointIndex;
  this->body = _rhs.body;
  this->geom1 = _rhs.geom1;
  this->geom2 = _rhs.geom2;

  return *this;
}

SimpleTrackedVehiclePlugin::ContactIterator
SimpleTrackedVehiclePlugin::ContactIterator::operator++(int /*_unused*/)
{
  ContactIterator i = *this;
  ++(*this);
  return i;
}

SimpleTrackedVehiclePlugin::ContactIterator::reference
SimpleTrackedVehiclePlugin::ContactIterator::operator*()
{
  if (!this->initialized)
    ++(*this);

  return *this->currentContact;
}

SimpleTrackedVehiclePlugin::ContactIterator::pointer
SimpleTrackedVehiclePlugin::ContactIterator::operator->()
{
  if (!this->initialized)
    ++(*this);

  return this->currentContact;
}

SimpleTrackedVehiclePlugin::ContactIterator::pointer
SimpleTrackedVehiclePlugin::ContactIterator::getPointer()
{
  if (!this->initialized)
    ++(*this);

  return this->currentContact;
}

bool SimpleTrackedVehiclePlugin::ContactIterator::operator!=(
    const SimpleTrackedVehiclePlugin::ContactIterator &_rhs)
{
  return !SimpleTrackedVehiclePlugin::ContactIterator::operator==(_rhs);
}
