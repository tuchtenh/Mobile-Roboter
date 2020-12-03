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
/*!\file    projects/rc_unimog_control_Unimog_v1/mObstacleDetctor.h
 *
 * \author  Aaron Hackenberg
 *
 * \date    2020-11-19
 *
 * \brief Contains mObstacleDetctor
 *
 * \b mObstacleDetctor
 *
 * This Module detects objects and notifies the main group when an object is detected.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_Unimog_v1__mObstacleDetctor_h__
#define __projects__rc_unimog_control_Unimog_v1__mObstacleDetctor_h__

#include "plugins/structure/tModule.h"
#include <vector>

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/si_units/si_units.h"
#include "rrlib/distance_data/tDistanceData.h"
#include "rrlib/math/tVector.h"
#include <string>
#include <tuple>


//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace rc_unimog_control_Unimog_v1
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This Module detects objects and notifies the main group when an object is detected.
 */
class mObstacleDetctor : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

	tInput<rrlib::distance_data::tDistanceData> input_point_cloud;

	tOutput<bool> out_object;
	tOutput<rrlib::si_units::tLength<>> distance;
	tOutput<double> angle;

	//tParameter<bool> enable;
/*
  tStaticParameter<double> static_parameter_1;   Example for a static parameter. Replace or delete it!

  tParameter<double> par_parameter_1;   Example for a runtime parameter named "Parameter 1". Replace or delete it!

  tInput<double> in_signal_1;   Example for input ports named "Signal 1" and "Signal 2". Replace or delete them!
  tInput<double> in_signal_2;

  tOutput<double> out_signal_1;   Examples for output ports named "Signal 1" and "Signal 2". Replace or delete them!
  tOutput<double> out_signal_2;
*/
//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mObstacleDetctor(core::tFrameworkElement *parent, const std::string &name = "ObstacleDetctor");
  void Closest_object();
//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  virtual ~mObstacleDetctor();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  //Here is the right place for your variables. Replace this line by your declarations!
  bool enable;

  rrlib::si_units::tLength<> current_length;

  double current_angle;

  std::vector< std::tuple<float, float> > close_points;

  virtual void OnStaticParameterChange() override;   //Might be needed to process static parameters. Delete otherwise!

  virtual void OnParameterChange() override;   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Update() override;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
