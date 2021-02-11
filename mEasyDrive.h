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
#include "projects/finroc_projects_robprak2020_2/easyDriveClass.h"

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
  tOutput<bool> out_noLineDetection;


  tInput<double> input_curvature_left;
  tInput<double> input_curvature_middle;
  tInput<double> input_curvature_right;


  tInput<bool> test_bool;


  tInput<bool> coneDetect;
  tInput<bool> switchToLeft;
  tInput<bool> switchToRight;

  tInput<int> straightOrLeft;

  tInput<bool> yellowSignDetect;



//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mEasyDrive(core::tFrameworkElement *parent, const std::string &name = "EasyDrive");

  void intitialization();
  void getDetectionAndManual();

  void ruleBaseAlgrithm();
  void linearAlgrithm();
  void powerAlgrithm(int distance, double pixelValue);


  void expAlgrithm(int distance, double pixelValue);
  void expStrongCurv(int distance, double pixelValue);

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

  enum ReactionState {EASY = 0, CONE, SWITCH_TO_LEFT_MANUAL, TURN_LEFT_GO_STRAIGHT_MANUAL, TURN_LEFT_GO_STRAIGHT_AUTO};
  ReactionState reactionState = EASY;


  double curvature;
  std::vector<bool> lights;


  virtual void OnStaticParameterChange() override;
  virtual void OnParameterChange() override;
  virtual void Update() override;




  LineDetMachine lineDet;
  LineDetMachine* lineDetPtr = &lineDet;

  IntersectDetMachine intersectDet;
  IntersectDetMachine* intersectDetPtr = &intersectDet;

  OverTakeDetMachine overTakeDet;
  OverTakeDetMachine* overTakeDetPtr = &overTakeDet;


  char straightOrLeft_int_temp;




};









//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
