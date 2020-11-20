//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    projects/rc_unimog_control_zed2/mObstacleDetector.h
 *
 * \author  Manuel Vogel
 *
 * \date    2020-11-19
 *
 * \brief Contains mObstacleDetector
 *
 * \b mObstacleDetector
 *
 * this module detects obstacles based on the depth info from zed2
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_zed2__mObstacleDetector_h__
#define __projects__rc_unimog_control_zed2__mObstacleDetector_h__

#include "plugins/structure/tModule.h"
#include "rrlib/si_units/si_units.h"
#include "rrlib/math/tAngle.h"
#include "rrlib/distance_data/tDistanceData.h"

#include "rrlib/math/angle/rtti.h"



//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace rc_unimog_control_zed2
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * this module detects obstacles based on the depth info from zed2
 */
class mObstacleDetector : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tInput<rrlib::distance_data::tDistanceData> input_point_cloud;
  tOutput<bool> object_in_way;
  //tOutput<rrlib::si_units::tLength<>> distance_to_object;
  //tOutput<rrlib::math::tAngleRad> angle_to_object;
  tOutput<double> distance_to_object;
  tOutput<double> angle_to_object;

/*  tStaticParameter<double> static_parameter_1;   Example for a static parameter. Replace or delete it!

  tParameter<double> par_parameter_1;   Example for a runtime parameter named "Parameter 1". Replace or delete it!

  tInput<double> in_signal_1;   Example for input ports named "Signal 1" and "Signal 2". Replace or delete them!

  tOutput<double> out_signal_1;   Examples for output ports named "Signal 1" and "Signal 2". Replace or delete them!
*/
//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mObstacleDetector(core::tFrameworkElement *parent, const std::string &name = "ObstacleDetector");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  virtual ~mObstacleDetector();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  bool object = false;

  int counter = 0;

  double distanceToObject = 100.0;

  double angleToObject = 0.0;

  virtual void OnStaticParameterChange() override;

  virtual void OnParameterChange() override;

  virtual void Update() override;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
