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
/*!\file    projects/rc_unimog_control_Unimog_v1/mImageHandler.cpp
 *
 * \author  Aaron Hackenberg
 *
 * \date    2020-11-26
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_Unimog_v1/mImageHandler.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <vector>
#include "rrlib/coviroa/opencv_utils.h"
#include "rrlib/color_blob_detection/cmvision.h"
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
//using namespace cv;
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
runtime_construction::tStandardCreateModuleAction<mImageHandler> cCREATE_ACTION_FOR_M_IMAGEHANDLER("ImageHandler");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mImageHandler constructor
//----------------------------------------------------------------------
mImageHandler::mImageHandler(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false)
{}

//----------------------------------------------------------------------
// mImageHandler destructor
//----------------------------------------------------------------------
mImageHandler::~mImageHandler()
{}

//----------------------------------------------------------------------
// mImageHandler OnStaticParameterChange
//----------------------------------------------------------------------
void mImageHandler::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mImageHandler OnParameterChange
//----------------------------------------------------------------------
void mImageHandler::OnParameterChange()
{

}

//----------------------------------------------------------------------
// mImageHandler Update
//----------------------------------------------------------------------
void mImageHandler::Update()
{
	/*if (this->in_image.HasChanged())
	    {
		data_ports::tPortDataPointer<const std::vector<rrlib::coviroa::tImage>> in_img = this->in_image.GetPointer();
		if (in_img->size() > 0) {
			const cv::Mat img = rrlib::coviroa::AccessImageAsMat(in_img->at(0));

			data_ports::tPortDataPointer<rrlib::coviroa::tImage> img_out = this->out_image.GetUnusedBuffer();
			rrlib::rtti::GenericOperations<rrlib::coviroa::tImage>::DeepCopy((*in_img)[0], *img_out);
			cv::Mat out = rrlib::coviroa::AccessImageAsMat(*img_out);

			out.convertTo(img, -1, 1, brightness.Get());
			// copy input image to output image
			cv::rectangle(out, cv::Point(200,200), cv::Point(400,400), cv::Scalar(0, 0, 255), 5, 8, 0);
			// use the tImage data acs an cv::Mat (does not copy the data)
			this->out_image.Publish(img_out);
		}



	    }*/
	if (this->in_images.HasChanged())
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
