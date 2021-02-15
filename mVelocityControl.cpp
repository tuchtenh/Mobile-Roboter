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
/*!\file    projects/finroc_projects_robprak2020_2/mVelocityControl.cpp
 *
 * \author  ChengYi Huang
 *
 * \date    2021-01-27
 *
 */
//----------------------------------------------------------------------
#include "projects/finroc_projects_robprak2020_2/mVelocityControl.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
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
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mVelocityControl> cCREATE_ACTION_FOR_M_VELOCITYCONTROL(
  "VelocityControl");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mVelocityControl constructor
//----------------------------------------------------------------------
mVelocityControl::mVelocityControl(core::tFrameworkElement *parent,
                                   const std::string &name) :
  tModule(parent, name, false) //, // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
//If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
{

}

//----------------------------------------------------------------------
// mVelocityControl destructor
//----------------------------------------------------------------------
mVelocityControl::~mVelocityControl()
{
}

//----------------------------------------------------------------------
// mVelocityControl OnStaticParameterChange
//----------------------------------------------------------------------
void mVelocityControl::OnStaticParameterChange()
{
  /*if (this->static_parameter_1.HasChanged())
   {
   //As this static parameter has changed, do something with its value!
   }*/
}

//----------------------------------------------------------------------
// mVelocityControl OnParameterChange
//----------------------------------------------------------------------
void mVelocityControl::OnParameterChange()
{
  //If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
}

//----------------------------------------------------------------------
// mVelocityControl Update
//----------------------------------------------------------------------
void mVelocityControl::Update()
{
  // initialization
  //turn = false;


  if (in_velocity.HasChanged())
  {
    maxVelocity = in_velocity.Get();
  }
  noLineDet = noLineDetEnable.Get();
  stopDet = stopEnable.Get();

  if (noLineDet)
  {
    out_velocity.Publish(0);
  }
  else if (unimogDet.Get())
  {
    out_velocity.Publish(0);
  }
  else if (stopDet || stopProcessOn)
  {
    out_velocity.Publish(reactToStopSign(stopDet));
  }
  else if (giveWayEnable.Get() || giveWayOn)
  {
    out_velocity.Publish(reactToGiveWaySign());
  }
  else
  {
    if (slowMode.Get())
    {
      out_velocity.Publish(minVelocity);
    }

    else if (singleMode.Get())
    {
      out_velocity.Publish(0.02);
      //velocityController On
    }
    else
    {
      out_velocity.Publish(maxVelocity);
    }
  }
  out_turn.Publish(turn);
}

//out_velocity.Publish(reactToStopSign(stop));

/*
 if (stop == true || stop_lock==true && drive_counter == 100)
 {

 out_velocity.Publish(0);
 stop_counter++;

 if(stop_counter == 50)
 {
 stop_lock == false;
 drive_counter = 0;
 }
 else
 {
 stop_lock == true;
 stop_counter = 0;
 }
 } else if (noLineDet == true)
 {
 out_velocity.Publish(0);
 }

 else
 {
 if (drive_counter < 100){
 drive_counter++;
 }
 out_velocity.Publish(0.8);
 }
 */

/*if (this->InputChanged())
 {
 At least one of your input ports has changed. Do something useful with its data.
 However, using the .HasChanged() method on each port you can check in more detail.
 }

 Do something each cycle independent from changing ports.

 this->out_signal_1.Publish(some meaningful value); can be used to publish data via your output ports.*/

double mVelocityControl::reactToStopSign(bool detectStop)
{
  double v;

  switch (stopSignState)
  {
  case INIT:
    v = maxVelocity;
    sc = 0;
    dc = 0;
    stopProcessOn = true;
    std::cout << "STOP sign Process On" << std::endl;

    stopSignState = STOP;

    break;

  case STOP:
    v = 0;
    sc++;

    if (sc <= scValue)
    {
      stopSignState = STOP;
    }
    else if (sc > scValue)
    {
      stopSignState = DRIVE;
    }
    else
    {
      std::cout << "STOP sign STOP state bug" << std::endl;
    }
    break;

  case DRIVE:
    dc++;
    v = maxVelocity;
    if (dc <= dcValue)
    {
      stopSignState = DRIVE;
    }
    else if (dc > dcValue)
    {
      stopSignState = INIT;
      stopProcessOn = false;
      std::cout << "STOP sign Process Off" << std::endl;
    }
    else
    {
      std::cout << "STOP sign DRIVE state bug" << std::endl;
    }
    break;

  default:
    std::cout << "STOP sign reaction wrong" << std::endl;
    break;

  }

  return v;
}

double mVelocityControl::reactToGiveWaySign()
{
  double v = 0;
  switch (ROWState)
  {
  case START:
    v = maxVelocity;
    ac = 0;
    ic = 0;
    giveWayOn = true;
    std::cout << "Give Way sign APPROACH On" << std::endl;

    ROWState = APPROACH;

    break;

  case APPROACH:
    v = maxVelocity - ((maxVelocity - minVelocity) * ac * 0.004);
    ac++;

    if (ac <= acValue)
    {
      ROWState = APPROACH;
    }
    else if (ac > acValue)
    {
      ROWState = CONTINUE;
    }
    else
    {
      std::cout << "Give Way sign APPROACH state bug" << std::endl;
    }
    break;

  case CONTINUE:
    ic++;
    v = maxVelocity;
    if (ic <= icValue)
    {
      ROWState = CONTINUE;
      turn = true;
    }
    else if (ic > icValue)
    {
      ROWState = START;
      giveWayOn = false;
      turn = false;
      std::cout << "Give Way sign APPROACH Off" << std::endl;
    }
    else
    {
      std::cout << "Give Way sign CONTINUE state bug" << std::endl;
    }
    break;

  default:
    std::cout << "Give Way sign reaction wrong" << std::endl;
    break;

  }
  return v;
}

double mVelocityControl::reactToBridge()
{
  double v = 0;
  switch (bridgeState)
  {
  case UP:
    v = minVelocity + cc * 0.02 * maxVelocity;
    cc++;
    if (cc <= ccValue)
    {
      bridgeProcessOn = true;
    }
    else if (cc > ccValue)
    {
      bridgeState = DOWN;
    }
    break;
  case DOWN:
    v = 0.1;
    desc++;
    if (desc > descValue)
    {
      bridgeState = UP;
      bridgeProcessOn = false;
    }
    break;
  default:
    v = 0;
    std::cout << "Climbing bridge reaction wrong" << std::endl;
    break;

  }
  return v;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
