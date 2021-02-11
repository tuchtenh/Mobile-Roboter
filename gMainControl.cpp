//
// You received this file as part of Finroc
// A framework for integrated robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    gMainControl.cpp
 *
 * \author  Unknown
 *
 * \date    2011-11-07
 *
 */
//----------------------------------------------------------------------
#include "projects/finroc_projects_robprak2020_2/gMainControl.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "projects/finroc_projects_robprak2020_2/mEasyDrive.h"

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
runtime_construction::tStandardCreateModuleAction<gMainControl> cCREATE_ACTION_FOR_G_MAIN_CONTROL("MainControl");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gMainControl constructors
//----------------------------------------------------------------------
gMainControl::gMainControl(finroc::core::tFrameworkElement *parent, const std::string &name,
                           const std::string &structure_config_file)
  : tSenseControlGroup(parent, name, structure_config_file)

  // to access hardware / unreal interface use this->hardware, e.g.
  // this->hardware->so_velocity.ConnectTo(...)
  // to set the velocity of the unimog: this->hardware->ci_velocity.ConnectTo(out_your_velocity_port);
  // etc.
{
  //create module
  //new mEasyDrive(this);
  mEasyDrive* easy_drive = new mEasyDrive(this);


  mZEDDetection* zed_detection = new mZEDDetection(this);
  //new mZEDDetection(this);

  mVelocityControl* velocity_control = new mVelocityControl(this);
  velocity_control->noLineDetEnable.ConnectTo(easy_drive->out_noLineDetection);
  /*
    mImageDetectorTKDNN*  imageDetector = new mImageDetectorTKDNN(this);
    imageDetector->stop.ConnectTo(velocity_control->stopEnable);
  */






  //ports
  //this->co_velocity.ConnectTo(easy_drive->out_velocity);
  //this->co_curvature.ConnectTo(easy_drive->out_curvature);
  //  this->hardware->ci_curvature.ConnectTo(easy_drive->out_curvature);
  //zed_detection->camera_in.ConnectTo(this->si_stereo_cam_image_0);//not working
  zed_detection->distance_to_mid_out.ConnectTo(easy_drive->input_curvature_middle);
  zed_detection->distance_to_right_out.ConnectTo(easy_drive->input_curvature_right);
  zed_detection->distance_to_left_out.ConnectTo(easy_drive->input_curvature_left);
  //this->hardware->so_cam_images.ConnectTo(zed_detection->camera_in);
  //this->co_velocity.ConnectTo(easy_drive->out_velocity);
  //this->co_curvature.ConnectTo(easy_drive->out_curvature);

  //zed_detection->block_move.ConnectTo(easy_drive->block_move_easyDrive);


}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
