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
/*!\file    projects/finroc_projects_robprak2020_2/mEasyDrive.h
 *
 * \author  Aaron Hackenberg
 *
 * \date    2020-12-03
 *
 * \brief Contains mEasyDrive
 *
 * \b mEasyDrive
 *
 * This module implements the most basic driving functions without obstacles.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__finroc_projects_robprak2020_2__mEasyDrive_h__
#define __projects__finroc_projects_robprak2020_2__mEasyDrive_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <vector>
#include <tuple>
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
class LineDetMachine
{

private:

  enum State { MID_LINE = 0 , RIGHT_LINE , LEFT_LINE , STOP };
  State state;
  double midValue = 0;
  double rightValue = 0;
  double leftValue = 0;

  double velocity = 0.4;
  double offset = 0;

  const double midValue_min = -100, midValue_max = 0;
  const double rightValue_min = 0, rightValue_max = 300;
  const double leftValue_min = -500, leftValue_max = -100;

  double x;



public:
  void setLineValue(double m, double r, double l);
  void chooseLine();
  double publishOffset();
  double publishPixel();

};


//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This module implements the most basic driving functions without obstacles.
 */
class mEasyDrive : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  //Ouput for the new velocity
  tOutput<double> out_velocity;
  //Ouput for the new curvature
  tOutput<double> out_curvature;
  //Ouput for the new light settings
  tOutput<std::vector<bool>> out_lights;


  //Input for the line error
  tInput<std::tuple<double, double, double>> line_error;



  //enable our instructions
  tInput<bool> enable;
  //determines time for the execution of one function
  tInput<int> time;
  //determines to drive straight or curve
  tInput<int> mode;

  tInput<double> line_error_test;

  //inputs for testing
  tInput<int> input_time_1;
  tInput<int> input_time_2;

  tInput<double> input_curvature_left;
  tInput<double> input_curvature_middle;
  tInput<double> input_curvature_right;

  tInput<double> input_velocity_1;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mEasyDrive(core::tFrameworkElement *parent, const std::string &name = "EasyDrive");
  //TODO add correct parameters
  void drive_straight();
  void drive_curve_1();
  void drive_curve_2();
  void drive_intersection();

  void ruleBaseAlgrithm();
  void linearAlgrithm();
  void powerAlgrithm();
  void expAlgrithm();
  void stop();

  void expAlgrithm(int offset, double pixelValue);

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  virtual ~mEasyDrive();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  bool curve;

  double velocity;

  double curvature;

  std::vector<bool> lights;

  int lock_checker;

  int unlock_value;

  bool lock;

  double right_boundary_1;
  double right_boundary_2;
  double right_boundary_3;
  double right_boundary_4;
  double right_boundary_5;


  virtual void OnStaticParameterChange() override;

  virtual void OnParameterChange() override;

  virtual void Update() override;


  LineDetMachine lineDet;


};









//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
