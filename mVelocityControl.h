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
/*!\file    projects/finroc_projects_robprak2020_2/mVelocityControl.h
 *
 * \author  ChengYi Huang
 *
 * \date    2021-01-27
 *
 * \brief Contains mVelocityControl
 *
 * \b mVelocityControl
 *
 * this module is responsible for controlling velocity. velocity=0 or velocity=constant value
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__finroc_projects_robprak2020_2__mVelocityControl_h__
#define __projects__finroc_projects_robprak2020_2__mVelocityControl_h__

#include "plugins/structure/tModule.h"

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
 * this module is responsible for controlling velocity. velocity=0 or velocity=constant value
 */
class mVelocityControl : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mVelocityControl(core::tFrameworkElement *parent, const std::string &name = "VelocityControl");

  tInput<bool> noLineDetEnable;
  tInput<bool> stopEnable;
  tInput<bool> conesDetEnable;
  tInput<bool> stop_5_secondEnable;
  tInput<bool> unimogDet;
  tInput<bool> rightOfWayEnable;
  tInput<double> frontSensor;
  tInput<bool> slowMode;

  tInput<double> in_velocity;
  tOutput<double> out_velocity;
  tOutput<bool> out_turn;



//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:


  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  virtual ~mVelocityControl();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  double minVelocity = 0.5;
  double maxVelocity = 0.8;

  bool noLineDet;

  //  STOP sign parameters   ////////////////////////////////////////////////////////////////////////////
  bool stopDet;
  bool stopProcessOn = false;

  bool rightOfWayOn = false;
  bool turn = false;

  bool bridgeProcessOn = false;

  enum StopSignState {INIT, STOP, DRIVE};
  StopSignState stopSignState = INIT;
  int sc = 0;
  int dc = 0;
  const int scValue = 50;
  const int dcValue = 30;

  enum RightOfWaySignState {START, APPROACH, CONTINUE};
  RightOfWaySignState ROWState = START;
  int ac = 0; //approach Counter
  int ic = 0; //ignore Counter to ignore the Sign;
  const int acValue = 250;
  const int icValue = 200;

  enum BridgeState {UP, DOWN};
  BridgeState bridgeState = UP;
  int cc = 0; //climb counter
  int desc = 0; //descend counter
  const int ccValue = 50;
  const int descValue = 200;

  double reactToStopSign(bool detectStop);
  double reactToRightOfWaySign();
  double reactToBridge();


  //////////////////////////////////////////////////////////////////////////////////////////////////////







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
