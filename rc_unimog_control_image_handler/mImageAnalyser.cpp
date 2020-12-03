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
/*!\file    projects/rc_unimog_control_image_handler/mImageAnalyser.cpp
 *
 * \author  Aaron Hackenberg
 *
 * \date    2020-11-26
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_image_handler/mImageAnalyser.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <opencv2/opencv.hpp>
#include "rrlib/coviroa/opencv_utils.h"
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
namespace rc_unimog_control_image_handler
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mImageAnalyser> cCREATE_ACTION_FOR_M_IMAGEANALYSER("ImageAnalyser");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mImageAnalyser constructor
//----------------------------------------------------------------------
mImageAnalyser::mImageAnalyser(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false)
{}

//----------------------------------------------------------------------
// mImageAnalyser destructor
//----------------------------------------------------------------------
mImageAnalyser::~mImageAnalyser()
{}

//----------------------------------------------------------------------
// mImageAnalyser OnStaticParameterChange
//----------------------------------------------------------------------
void mImageAnalyser::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mImageAnalyser OnParameterChange
//----------------------------------------------------------------------
void mImageAnalyser::OnParameterChange()
{

}

//----------------------------------------------------------------------
// mImageAnalyser Update
//----------------------------------------------------------------------
void mImageAnalyser::Update()
{
	if (this->input_images.HasChanged())
	  {
	   data_ports::tPortDataPointer<const std::vector<rrlib::coviroa::tImage>> in_img = this->input_images.GetPointer();

	     if (in_img->size() > 0)
	     {
	         //opencvImage.convertTo(imageBrighnessHigh50, -1, 1, brightness.Get());//brightness.Get());

	         data_ports::tPortDataPointer<rrlib::coviroa::tImage> img_out = this->output_image.GetUnusedBuffer();
	         rrlib::rtti::GenericOperations<rrlib::coviroa::tImage>::DeepCopy((*in_img)[0], *img_out);

	         /*CMVision* vision = new CMVision();
	         vision->initialize(640,480);
	         vision->classifyFrame("etc/image_0.png", "etc/cmvision_color_lut_combined.bin");
	         vision->loadOptions("etc/colors.txt");
	         vision->PrintLookUpTable();
	         vision->PrintCustomLookUpTable();*/
	         cv::Mat img = rrlib::coviroa::AccessImageAsMat(*img_out);
	         img.convertTo(img, -1, 1, brightness.Get());
	         cv::rectangle(img, cv::Point(100,100),cv::Point(400,400),cv::Scalar(0,255,0));
	         cv::Mat out = rrlib::coviroa::AccessImageAsMat(*img_out);
	         this->output_image.Publish(img_out);
	    }
	  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
