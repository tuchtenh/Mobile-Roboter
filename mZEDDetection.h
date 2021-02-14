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
/*!\file    projects/rc_unimog_control_ub5/mZEDDetection.h
 *
 * \author  Lukas Tuchtenhagen
 *
 * \date    2020-12-03
 *
 * \brief Contains mZEDDetection
 *
 * \b mZEDDetection
 *
 * classification and detection of objects using the ZED camera
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_ub5__mZEDDetection_h__
#define __projects__rc_unimog_control_ub5__mZEDDetection_h__

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
namespace finroc_projects_robprak2020_2
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

  //enable our instructions
  //tInput<bool> enable;
  tInput<std::vector<rrlib::coviroa::tImage>> camera_in;
  tOutput<rrlib::coviroa::tImage> camera_out;
  tOutput<double> distance_to_left_out;
  tOutput<double> distance_to_mid_out;
  tOutput<double> distance_to_right_out;
  tOutput<double> angle_to_left_out;
  tOutput<double> angle_to_mid_out;
  tOutput<double> angle_to_right_out;

  tInput<bool> blueOrRed;
  tInput<bool> colorSwitchEnable;
  tInput<bool> colorSwitchFromEasy;

  tOutput<bool> gui_colorSwitch;

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


  //virtual std::vector<double> MidDetection();
  //virtual std::vector<double> SideDetection();

  virtual void OnParameterChange() override;
  virtual void Update() override;

  std::queue <double> queueRight;
  std::queue <double> queueMid;
  std::queue <double> queueLeft;

  double left_red_x = 0;
  double right_red_x = 672;
  double left_red_y = 300;
  double right_red_y = 300;

  double left_white_x = 170;
  double right_white_x = 380;
  double left_white_y = 330;
  double right_white_y = 330;

  double red_detection_threshold = 350;

  double gray_treshold = 170;

  //double M = -70;
  //double m = 0.95;
  double a = 0;
  double old_right;
  double old_mid;
  double old_left;
  double contrast = 1;
  bool contrast_change = true;

  enum ColorState {BLUE = 0, RED};
  ColorState colorState = BLUE;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
