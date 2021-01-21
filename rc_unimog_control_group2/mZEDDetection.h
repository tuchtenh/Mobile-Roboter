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
/*!\file    projects/finroc_projects_robprak2020_2/rc_unimog_control_group2/mZEDDetection.h
 *
 * \author  Lukas Tuchtenhagen
 *
 * \date    2020-12-07
 *
 * \brief Contains mZEDDetection
 *
 * \b mZEDDetection
 *
 * Detects any line and measures the distance between picturecenter and line
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__finroc_projects_robprak2020_2__rc_unimog_control_group2__mZEDDetection_h__
#define __projects__finroc_projects_robprak2020_2__rc_unimog_control_group2__mZEDDetection_h__


#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/coviroa/tImage.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "rrlib/coviroa/opencv_utils.h"
#include <opencv2/core/matx.hpp>
#include <queue>
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace rc_unimog_control_group2
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * classification and detection of objects using the ZED camera
 */
class mZEDDetection : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tInput<std::vector<rrlib::coviroa::tImage>> camera_in;
  tOutput<rrlib::coviroa::tImage> camera_out;
  tOutput<double> distance_to_left_out;
  tOutput<double> distance_to_mid_out;
  tOutput<double> distance_to_right_out;
  /*
   * angle values of 90° men the robot parallel to the line
   *  angles lower than 90° mean the robot is rotate to the right
   *  angles higher than 90° mean the robot is rotate to the left
   */
  tOutput<double> angle_to_left_out;
  tOutput<double> angle_to_mid_out;
  tOutput<double> angle_to_right_out;



//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mZEDDetection(core::tFrameworkElement *parent, const std::string &name = "ZEDDetection");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  virtual ~mZEDDetection();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:


  virtual std::vector<double> MidDetection();
  virtual std::vector<double> SideDetection();

  virtual void OnParameterChange() override;
  virtual void Update() override;

  std::queue <double> queueRight;
  std::queue <double> queueMid;
  std::queue <double> queueLeft;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
