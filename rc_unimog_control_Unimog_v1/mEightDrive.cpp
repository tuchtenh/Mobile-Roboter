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
/*!\file    projects/rc_unimog_control_Unimog_v1/mEightDrive.cpp
 *
 * \author  Aaron Hackenberg
 *
 * \date    2020-11-12
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_Unimog_v1/mEightDrive.h"

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
namespace rc_unimog_control_Unimog_v1
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mEightDrive> cCREATE_ACTION_FOR_M_EIGHTDRIVE("EightDrive");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mEightDrive constructor
//----------------------------------------------------------------------
mEightDrive::mEightDrive(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false), // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
  velocity(0),
  curvature(0),
  check(0),
  wanted_curvature(4)
{}

//----------------------------------------------------------------------
// mEightDrive destructor
//----------------------------------------------------------------------
mEightDrive::~mEightDrive()
{}

//----------------------------------------------------------------------
// mEightDrive OnStaticParameterChange
//----------------------------------------------------------------------
void mEightDrive::OnStaticParameterChange()
{
}

//----------------------------------------------------------------------
// mEightDrive OnParameterChange
//----------------------------------------------------------------------
void mEightDrive::OnParameterChange()
{

}

//----------------------------------------------------------------------
// mEightDrive Update
//----------------------------------------------------------------------
void mEightDrive::Update()
{
	//works with check >= 810 and cu = 1
	//works with check >= 515 and cu = 2
	//works with check >= 398 and cu = 3
	//works with check >= 340 and cu = 4
	//works with check >= 310 and cu = 5
  check = check + 1;
  if (check >= 340) {
	  check = 0;
	  wanted_curvature = -wanted_curvature;
  }
  //Do something each cycle independent from changing ports.
  this->velocity.Publish(2);
  this->curvature.Publish(wanted_curvature);
  this->check_value.Publish(check);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
