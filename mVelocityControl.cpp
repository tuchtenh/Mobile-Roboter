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
runtime_construction::tStandardCreateModuleAction<mVelocityControl> cCREATE_ACTION_FOR_M_VELOCITYCONTROL("VelocityControl");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mVelocityControl constructor
//----------------------------------------------------------------------
mVelocityControl::mVelocityControl(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false)//, // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
{



}

//----------------------------------------------------------------------
// mVelocityControl destructor
//----------------------------------------------------------------------
mVelocityControl::~mVelocityControl()
{}

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
  noLineDet = noLineDetEnable.Get();
  stop = stopEnable.Get();

  if (noLineDet == true)
  {
    out_velocity.Publish(0);
  }

  else if (stop == true)
  {

    out_velocity.Publish(0);
  }

  else
  {
    out_velocity.Publish(0.8);
  }
  /*if (this->InputChanged())
  {
    At least one of your input ports has changed. Do something useful with its data.
    However, using the .HasChanged() method on each port you can check in more detail.
  }

  Do something each cycle independent from changing ports.

  this->out_signal_1.Publish(some meaningful value); can be used to publish data via your output ports.*/
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
