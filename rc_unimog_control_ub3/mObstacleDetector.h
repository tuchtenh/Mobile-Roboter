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
/*!\file    projects/rc_unimog_control_ub3/mObstacleDetector.h
 *
 * \author  Lukas Tuchtenhagen
 *
 * \date    2020-11-18
 *
 * \brief Contains mObstacleDetector
 *
 * \b mObstacleDetector
 *
 * gives a boolean value if it detects an object
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_ub3__mObstacleDetector_h__
#define __projects__rc_unimog_control_ub3__mObstacleDetector_h__

#include "plugins/structure/tModule.h"
#include "rrlib/distance_data/tDistanceData.h"

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
namespace rc_unimog_control_ub3
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * gives a boolean value if it detects an object
 */
class mObstacleDetector : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

    /*
  tStaticParameter<double> static_parameter_1;   Example for a static parameter. Replace or delete it!

  tParameter<double> par_parameter_1;   Example for a runtime parameter named "Parameter 1". Replace or delete it!

  tInput<double> in_signal_1;   Example for input ports named "Signal 1" and "Signal 2". Replace or delete them!
  tInput<double> in_signal_2;

  tOutput<double> out_signal_1;   Examples for output ports named "Signal 1" and "Signal 2". Replace or delete them!
  tOutput<double> out_signal_2;
*/
    tOutput<double> distance_out;
    tOutput<double> angle_out;
    tOutput<bool> boolOutput;
    tInput<rrlib::distance_data::tDistanceData> input_point_cloud;

    //data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> cloud;
    
    //tParameter<const rrlib::distance_data::tDistanceData> cloud;
    
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

  //Here is the right place for your variables. Replace this line by your declarations!

    
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
