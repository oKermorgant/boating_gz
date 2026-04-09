/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include <gz/msgs/param.pb.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>

#include <gz/sim/System.hh>
#include <sdf/sdf.hh>
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/World.hh"
#include "gz/transport/Node.hh"

#include "Wavefield.hh"

using namespace gz;

namespace boating
{

/// \brief Internal Hull data
struct HullImplementation
{  
  /// \brief Parse the points via SDF.
  /// \param[in] _sdf Pointer to the SDF.
  void ParsePoints(const std::shared_ptr<const sdf::Element> &_sdf)
  {
    if (!_sdf->HasElement("points"))
      return;

    auto ptr = const_cast<sdf::Element *>(_sdf.get());
    auto sdfPoints = ptr->GetElement("points");

    // We need at least one point.
    if (!sdfPoints->HasElement("point"))
      gzerr << "Unable to find <points><point> element in SDF." << std::endl;

    auto pointElem = sdfPoints->GetElement("point");

    // Parse a new point.
    while (pointElem)
    {
      math::Vector3d point;
      pointElem->GetValue()->Get<math::Vector3d>(point);
      this->points.push_back(point);

      // Parse the next point.
      pointElem = pointElem->GetNextElement("point");
    }
  }

  /// \brief Callback for receiving wave field updates.
  /// \param[in] _msg The message containing all the wave field parameters.
  void OnWavefield(const msgs::Param &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->wavefield.Load(_msg);
  }

  /// \brief The link entity.
  sim::Link link{sim::kNullEntity};

  /// \brief Vessel length [m].
  double hullLength = 4.9;

  /// \brief Demi-hull radius [m].
  double hullRadius = 0.213;

  /// \brief Fluid height [m].
  double fluidLevel = 0;

  /// \brief Fluid density [kg/m^3].
  double fluidDensity = 997.7735;

  /// \brief The world's gravity [m/s^2].
  math::Vector3d gravity;

  /// \brief The points where the plugin applies forces. These points are
  /// relative to the link paramter's origin. Note that we don't check that the
  /// points are contained within the hull. You should pass reasonable points.
  std::vector<math::Vector3d> points;

  /// \brief The wavefield.
  Wavefield wavefield;

  /// \brief A node for receiving wavefield updates.
  transport::Node node;

  /// \brief Mutex to protect the wavefield.
  std::mutex mutex;
};


/// \brief A system that simulates the buoyancy of an object at the surface of
/// a fluid. This system must be attached to a model and the system will apply
/// buoyancy to a collection of points around a given link.
///
/// This system models the vehicle's buoyancy assuming a single hull with a
/// cylindrical shape. It's possible to derive from this plugin and provide
/// your own buoyancy function at each point. For this purpose you
/// should override `BuoyancyAtPoint()` in the derived plugin.
///
/// This plugin also supports waves. If you provide a wavefield via SDF, the
/// plugin will account for the delta Z that the waves generate at each point.
///
/// ## Required system parameters
///
/// * `<link_name>` is the name of the link used to apply forces.
///
/// ## Optional system parameters
///
/// * `<vehicle_length>` is the length of the vessel [m].
/// * `<hull_radius>` is the radius of the vessel's hull [m].
/// * `<fluid_level>` is the depth at which the fluid should be in the vehicle
/// * `<fluid_density>` is the density of the fluid.
/// * `<points>` contains a collection of points where the forces generated
///              by this plugin will be applied. See the format of each point
///              next:
/// *   `<point><position>` Relative position of the point relative to
///                         `link_name`.
/// * <wavefield>: The wavefield parameters. See `Wavefield.hh`.
///
/// ## Example
/// <plugin
///   filename="boating-Hull"
///   name="boating::Hull">
///   <link_name>base_link</link_name>
///   <vehicle_length>4.9</vehicle_length>
///   <vehicle_width>2.4</vehicle_width>
///   <hull_radius>0.213</hull_radius>
///   <fluid_level>0</fluid_level>

///   <!-- Points -->
///   <points>
///     <point>
///       <position>1.225 1.2 0</position>
///     </point>
///     <point>
///       <position>1.225 -1.2 0</position>
///     </point>
///     <point>
///       <position>-1.225 1.2 0</position>
///     </point>
///     <point>
///       <position>-1.225 -1.2 0</position>
///     </point>
///   </points>

///   <!-- Waves -->
///   <wavefield>
///     <size><%= $wavefield_size%> <%= $wavefield_size%></size>
///     <cell_count><%= $wavefield_cell_count%> <%=$wavefield_cell_count%></cell_count>
///     <wave>
///       <model>PMS</model>
///       <period>5.0</period>
///       <number>3</number>
///       <scale>1.1</scale>
///       <gain>0.5</gain>
///       <direction>1 0</direction>
///       <angle>0.4</angle>
///       <tau>2.0</tau>
///       <amplitude>0.0</amplitude>
///       <steepness>0.0</steepness>
///     </wave>
///   </wavefield>
/// </plugin>


class Hull
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{

private: std::unique_ptr<HullImplementation> dataPtr{std::make_unique<HullImplementation>()};

public:
  Hull() : gz::sim::System()
  {}

  ~Hull() override = default;


  void Configure(const sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 sim::EntityComponentManager &_ecm,
                 sim::EventManager &/*_eventMgr*/) override
  {
    // Parse required elements.
    if (!_sdf->HasElement("link_name"))
    {
      gzerr << "No <link_name> specified" << std::endl;
      return;
    }

    sim::Model model(_entity);
    std::string linkName = _sdf->Get<std::string>("link_name");
    this->dataPtr->link = sim::Link(model.LinkByName(_ecm, linkName));
    if (!this->dataPtr->link.Valid(_ecm))
    {
      gzerr << "Could not find link named [" << linkName
            << "] in model" << std::endl;
      return;
    }

    // Optional parameters.
    // Although some of these parameters are required in this plugin, a potential
    // derived plugin might not need them. Make sure that the default values are
    // reasonable.
    if (_sdf->HasElement("hull_length"))
    {
      this->dataPtr->hullLength = _sdf->Get<double>("hull_length");
    }

    if (_sdf->HasElement("hull_radius"))
    {
      this->dataPtr->hullRadius = _sdf->Get<double>("hull_radius");
    }

    if (_sdf->HasElement("fluid_level"))
    {
      this->dataPtr->fluidLevel = _sdf->Get<double>("fluid_level");
    }

    if (_sdf->HasElement("fluid_density"))
    {
      this->dataPtr->fluidDensity = _sdf->Get<double>("fluid_density");
    }

    // Parse the optional <points> element.
    this->dataPtr->ParsePoints(_sdf);

    // Get the gravity from the world.
    auto worldEntity = sim::worldEntity(_ecm);
    auto world = sim::World(worldEntity);
    auto gravityOpt = world.Gravity(_ecm);
    if (!gravityOpt)
    {
      gzerr << "Unable to get the gravity from the world" << std::endl;
      return;
    }
    this->dataPtr->gravity = *gravityOpt;

    // Wavefield
    this->dataPtr->wavefield.Load(_sdf);

    gzdbg << "Surface plugin successfully configured with the following "
          << "parameters:" << std::endl;
    gzdbg << "  <link_name>: " << linkName << std::endl;
    gzdbg << "  <vehicle_length>: " << this->dataPtr->hullLength << std::endl;
    gzdbg << "  <hull_radius>: " << this->dataPtr->hullRadius << std::endl;
    gzdbg << "  <fluid_level>: " << this->dataPtr->fluidLevel << std::endl;
    gzdbg << "  <fluid_density>: " << this->dataPtr->fluidDensity << std::endl;
    gzdbg << "  <points>:" << std::endl;
    for (const auto &p : this->dataPtr->points)
      gzdbg << "    [" << p << "]" << std::endl;

    // Subscribe to receive wavefield parameters.
    this->dataPtr->node.Subscribe(this->dataPtr->wavefield.Topic(),
                                  &HullImplementation::OnWavefield, this->dataPtr.get());
  }

  //////////////////////////////////////////////////
  void PreUpdate(const sim::UpdateInfo &_info,
                 sim::EntityComponentManager &_ecm) override
  {
    GZ_PROFILE("Surface::PreUpdate");

    if (_info.paused)
      return;

    // Vehicle frame transform.
    const auto kPose = this->dataPtr->link.WorldPose(_ecm);
    if (!kPose)
    {
      gzerr << "Unable to get world pose from link ["
            << this->dataPtr->link.Entity() << "]" << std::endl;
      return;
    }
    const math::Vector3d kEuler = (*kPose).Rot().Euler();
    math::Quaternion vq(kEuler.X(), kEuler.Y(), kEuler.Z());

    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    for (auto const &bpnt : this->dataPtr->points)
    {
      // Transform from vessel to fluid/world frame.
      const math::Vector3d kBpntW = vq * bpnt;

      // Vertical location of boat grid point in world frame.
      const float kDdz = (*kPose).Pos().Z() + kBpntW.Z();

      // World location of grid point.
      math::Vector3d point;
      point.X() = (*kPose).Pos().X() + kBpntW.X();
      point.Y() = (*kPose).Pos().Y() + kBpntW.Y();

      // Compute the depth at the grid point.
      double simTime = std::chrono::duration<double>(_info.simTime).count();
      double depth = this->dataPtr->wavefield.ComputeDepthSimply(point, simTime);

      // Vertical wave displacement.
      double dz = depth + point.Z();

      // Total z location of boat grid point relative to fluid surface.
      double deltaZ = (this->dataPtr->fluidLevel + dz) - kDdz;
      // Enforce only upward buoy force
      deltaZ = std::max(deltaZ, 0.0);
      deltaZ = std::min(deltaZ, this->dataPtr->hullRadius);

      const float kBuoyForce =
          this->CircleSegment(this->dataPtr->hullRadius, deltaZ) *
          this->dataPtr->hullLength / this->dataPtr->points.size() *
          -this->dataPtr->gravity.Z() * this->dataPtr->fluidDensity;

      // Apply force at the point.
      // Position is in the link frame and force is in world frame.
      this->dataPtr->link.AddWorldForce(_ecm,
                                        math::Vector3d(0, 0, kBuoyForce),
                                        bpnt);

      // Debug output:
      // gzdbg << bpnt.X() << "," << bpnt.Y() << "," << bpnt.Z() << std::endl;
      // gzdbg << "depth: " << depth << std::endl;
      // gzdbg << "dz: " << dz << std::endl;
      // gzdbg << "kDdz: " << kDdz << std::endl;
      // gzdbg << "deltaZ: " << deltaZ << std::endl;
      // gzdbg << "hull radius: " << this->dataPtr->hullRadius << std::endl;
      // gzdbg << "vehicle length: " << this->dataPtr->hullLength << std::endl;
      // gzdbg << "gravity z: " << -this->dataPtr->gravity.Z() << std::endl;
      // gzdbg << "fluid density: " << this->dataPtr->fluidDensity << std::endl;
      // gzdbg << "Force: " << kBuoyForce << std::endl << std::endl;
    }
  }

  //////////////////////////////////////////////////
  math::Vector3d Gravity() const
  {
    return this->dataPtr->gravity;
  }

  //////////////////////////////////////////////////
  double HullLength() const
  {
    return this->dataPtr->hullLength;
  }

  //////////////////////////////////////////////////
  double HullRadius() const
  {
    return this->dataPtr->hullRadius;
  }

  //////////////////////////////////////////////////
  double FluidDensity() const
  {
    return this->dataPtr->fluidDensity;
  }

  //////////////////////////////////////////////////
  double CircleSegment(double _r, double _h) const
  {
    return _r * _r * acos((_r -_h) / _r ) -
           (_r - _h) * sqrt(2 * _r * _h - _h * _h);
  }

};

}

GZ_ADD_PLUGIN(boating::Hull,
              sim::System,
              sim::ISystemConfigure,
              sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(boating::Hull,
                    "boating::Hull")
