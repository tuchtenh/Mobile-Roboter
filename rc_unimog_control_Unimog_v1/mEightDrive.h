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
/*!\file    projects/rc_unimog_control_Unimog_v1/mEightDrive.h
 *
 * \author  Aaron Hackenberg
 *
 * \date    2020-11-12
 *
 * \brief Contains mEightDrive
 *
 * \b mEightDrive
 *
 * This module is hopefully the one working!
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_Unimog_v1__mEightDrive_h__
#define __projects__rc_unimog_control_Unimog_v1__mEightDrive_h__

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
namespace rc_unimog_control_Unimog_v1
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This module is hopefully the one working!
 */
class mEightDrive : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tInput<bool> light;

  tOutput<double> velocity;
  tOutput<double> curvature;
  tOutput<int> check_value;

  /*tOutput<double> out_signal_1;   Examples for output ports named "Signal 1" and "Signal 2". Replace or delete them!
  tOutput<double> out_signal_2;
*/
//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mEightDrive(core::tFrameworkElement *parent, const std::string &name = "EightDrive");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  virtual ~mEightDrive();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  int check;

  double wanted_curvature;

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
