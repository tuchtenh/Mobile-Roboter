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
/*!\file    projects/rc_unimog_control_zed2/mImageManipulator.cpp
 *
 * \author  Manuel Vogel
 *
 * \date    2020-11-24
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_imageManipulation/mImageManipulator.h"
#include <opencv2/opencv.hpp>
#include "rrlib/coviroa/opencv_utils.h"

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
namespace rc_unimog_control_imageManipulation
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mImageManipulator> cCREATE_ACTION_FOR_M_IMAGEMANIPULATOR("ImageManipulator");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mImageManipulator constructor
//----------------------------------------------------------------------
mImageManipulator::mImageManipulator(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false)
  //output_image(rrlib::coviroa::tImage image(640, 480, eIMAGE_FORMAT_RGB24, 0)) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
{}

//----------------------------------------------------------------------
// mImageManipulator destructor
//----------------------------------------------------------------------
mImageManipulator::~mImageManipulator()
{}

//----------------------------------------------------------------------
// mImageManipulator OnStaticParameterChange
//----------------------------------------------------------------------
void mImageManipulator::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mImageManipulator OnParameterChange
//----------------------------------------------------------------------
void mImageManipulator::OnParameterChange()
{
  //If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
}

//----------------------------------------------------------------------
// mImageManipulator Update
//----------------------------------------------------------------------
void mImageManipulator::Update()
{

  if (this->input_images.HasChanged())
  {
   data_ports::tPortDataPointer<const std::vector<rrlib::coviroa::tImage>> in_img = this->input_images.GetPointer();

     if (in_img->size() > 0)
     {
         //cv::Mat imageBrighnessHigh50;
         //opencvImage.convertTo(imageBrighnessHigh50, -1, 1, brightness.Get());//brightness.Get());

         data_ports::tPortDataPointer<rrlib::coviroa::tImage> img_out = this->output_image.GetUnusedBuffer();
         rrlib::rtti::GenericOperations<rrlib::coviroa::tImage>::DeepCopy((*in_img)[0], *img_out);

         cv::Mat opencvImage = rrlib::coviroa::AccessImageAsMat(*img_out);
	 
         opencvImage = opencvImage + cv::Scalar(brightness.Get(),brightness.Get(),brightness.Get());
         cv::rectangle(opencvImage, cv::Point(200,200),cv::Point(350,500),cv::Scalar(255,0,255));
         cv::Mat out = rrlib::coviroa::AccessImageAsMat(*img_out);
         img_out.SetTimestamp(in_img.GetTimestamp());
         this->output_image.Publish(img_out);
    }
  }
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
