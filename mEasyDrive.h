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


protected:

  //enum State { MID_LINE = 0 , RIGHT_LINE , LEFT_LINE , STOP };
  //State state = MID_LINE;

  enum State {  RIGHT_LINE = 0 , LEFT_LINE , STOP };
  State state = RIGHT_LINE;

  double midValue = 0;
  double rightValue = 0;
  double leftValue = 0;

  double distance = 0;

  const double midValue_min = -195, midValue_max = 0; // Max needs tuned
  const double rightValue_min = 0, rightValue_max = 470;
  const double leftValue_min = -405, leftValue_max = -120; // Max needs tuned


//test/////////////////////////////////////////////////////////////
  /*
    const double midValue_min = -200, midValue_max = 200;
      const double rightValue_min = -500, rightValue_max = 500;
      const double leftValue_min = -600, leftValue_max = 600;

  */
//test/////////////////////////////////////////////////////////////


  /*//previous
  const double midValue_min = -130, midValue_max = 0;
  const double rightValue_min = 0, rightValue_max = 300;
  const double leftValue_min = -700, leftValue_max = -120;
  */
  const double rLane_mid_distance = 70;
  const double rLane_right_distance = -185;
  const double rLane_left_distance = 325;
  /*
  const double rLane_mid_distance = 67;
    const double rLane_right_distance = -155;
    const double rLane_left_distance = -295;
  */

  double pixel;

  bool noLineDetection = false;

public:
  void setLineValue(double m, double r, double l);
  virtual void chooseLine();
  double publishDistance();
  double publishPixel();
  bool   publishNoDetect();
  std::tuple<int, double, bool> operation(double m, double r, double l);





};

class IntersectDetMachine : public LineDetMachine
{
private:

  int mc = 0;
  int rc = 0;
  const int interCounter = 50;

  enum IntersectState { MID_LINE = 0, RIGHT_LINE, LEFT_LINE, STOP };
  IntersectState intersectState = MID_LINE;
  IntersectState stateMemory = MID_LINE;


public:
  bool interProcessOn = false;
  void chooseLine();
};

class OverTakeDetMachine : public LineDetMachine
{
private:

  enum OverTakeState { LEFT_LINE = 0, RIGHT_LINE, STOP };
  OverTakeState overTakeState = LEFT_LINE;

  //const double lLane_mid_distance = -155;
  const double lLane_right_distance = -330;
  const double lLane_left_distance = 176;

  //enum OvertakeState {SWITCH_LEFT, KEEP_DRIVING, STOP};
  //OvertakeState overtakeState = SWITCH_LEFT;

  //const double lMidValue_min = -200, lMidValue_max = 200;
  const double lRightValue_min = 70, lRightValue_max = 490;
  const double lLeftValue_min = -420, lLeftValue_max = -9;

public:
  bool takeoverProcessOn = false;
  void chooseLine();
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


  tOutput<double> out_curvature;
  //Ouput for the new light settings
  tOutput<std::vector<bool>> out_lights;

  tOutput<bool> out_noLineDetection;


  tInput<double> input_curvature_left;
  tInput<double> input_curvature_middle;
  tInput<double> input_curvature_right;


  tInput<bool> test_bool;
  //tInput<double> input_velocity_1;

  tOutput<double> block_move_easyDrive;

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
  void powerAlgrithm(int distance, double pixelValue);

  void stop();

  void expAlgrithm(int distance, double pixelValue);

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

  int test_counter = 0 ;


  virtual void OnStaticParameterChange() override;

  virtual void OnParameterChange() override;

  virtual void Update() override;




  LineDetMachine lineDet;
  LineDetMachine* lineDetPtr = &lineDet;

  IntersectDetMachine intersectDet;
  IntersectDetMachine* intersectDetPtr = &intersectDet;

  OverTakeDetMachine overTakeDet;
  OverTakeDetMachine* overTakeDetPtr = &overTakeDet;



};









//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
