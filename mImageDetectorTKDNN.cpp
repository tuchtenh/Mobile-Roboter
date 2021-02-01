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
/*!\file    projects/finroc_projects_robprak2020_2/mImageDetectorTKDNN.cpp
 *
 * \author  Manuel Vogel
 *
 * \date    2021-01-15
 *
 */
//----------------------------------------------------------------------
#include "projects/finroc_projects_robprak2020_2/mImageDetectorTKDNN.h"
//#include <image.h>
#include <vector>
//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "rrlib/coviroa/opencv_utils.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


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
runtime_construction::tStandardCreateModuleAction<mImageDetectorTKDNN> cCREATE_ACTION_FOR_M_IMAGEDETECTORTKDNN("ImageDetectorTKDNN");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mImageDetectorTKDNN constructor
//----------------------------------------------------------------------
mImageDetectorTKDNN::mImageDetectorTKDNN(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
{
  detNN = &yolo;

  int n_classes = 5;
  n_batch = 1;
  detNN->init("/home/agrosy/Documents/tkDNN/build/Unimog_fp32.rt", n_classes, n_batch, conf_thresh);


}

//----------------------------------------------------------------------
// mImageDetectorTKDNN destructor
//----------------------------------------------------------------------
mImageDetectorTKDNN::~mImageDetectorTKDNN()
{}

//----------------------------------------------------------------------
// mImageDetectorTKDNN OnStaticParameterChange
//----------------------------------------------------------------------
void mImageDetectorTKDNN::OnStaticParameterChange()
{
}

//----------------------------------------------------------------------
// mImageDetectorTKDNN OnParameterChange
//----------------------------------------------------------------------
void mImageDetectorTKDNN::OnParameterChange()
{


}

//----------------------------------------------------------------------
// mImageDetectorTKDNN Update
//----------------------------------------------------------------------
void mImageDetectorTKDNN::Update()
{

  if (this->in_images.HasChanged())
  {
    data_ports::tPortDataPointer<const std::vector<rrlib::coviroa::tImage>> in_img = this->in_images.GetPointer();
    // check if the vector really contains at least one image
    if (in_img->size() > 0)
    {


      cv::Mat cam_image = rrlib::coviroa::AccessImageAsMat(in_img->at(0));
      cv::Mat cam_image_img;
      cvtColor(cam_image, cam_image_img, CV_BGR2RGB);

      bool stopDetect = false, give_wayDetect = false, conesDetect = false, right_of_wayDetect = false, unimogDetect  = false;
      double stopSize = 0.0, give_waySize = 0.0, conesSize = 0.0, right_of_waySize = 0.0, unimogSize = 0.0;

      batch_dnn_input.clear();
      batch_frame.clear();

      batch_dnn_input.push_back(cam_image_img.clone());
      batch_frame.push_back(cam_image_img);
      detNN->update(batch_dnn_input, n_batch);

      detNN->draw(batch_frame);
      detected_bbox.clear();
      detected_bbox = detNN->detected;

      for (auto d : detected_bbox)
      {
        //std::cout<<"found something with class"<<d.cl<<"size:"<<d.w<<" x " <<d.h<<"results:"<<d.h*d.w<<std::endl;
        switch (d.cl)
        {
        case 0: //stop
          stopSize = d.w * d.h;
          if (stopSize > 1800)
          {
            stopDetect = true;

          }
          break;
        case 1:
          right_of_waySize = d.w * d.h;
          if (right_of_waySize > 650)
          {
            right_of_wayDetect = true;
          }
          break;
        case 2:
          give_waySize = d.w * d.h;
          if (give_waySize > 1200)
          {
            give_wayDetect = true;

          }
          break;
        case 3:
          conesSize = d.w * d.h;
          if (conesSize > 1000)
          {
            conesDetect = true;
          }
          break;
        case 4://unimog
          unimogSize = d.w * d.h;
          if (unimogSize > 5000)
          {
            unimogDetect = true;
          }
          break;
        }
      }
      //std::cout<<"still working"<<std::endl;
      //cv::imwrite("detection.png", batch_frame[0]);
      // prepare output image
      data_ports::tPortDataPointer<rrlib::coviroa::tImage> img_out = this->out_image.GetUnusedBuffer();
      // copy input image to output image
      rrlib::rtti::GenericOperations<rrlib::coviroa::tImage>::DeepCopy((*in_img)[0], *img_out);
      // use the tImage data acs an cv::Mat (does not copy the data)
      cv::Mat out = rrlib::coviroa::AccessImageAsMat(*img_out);
      out = batch_frame[0];

      img_out.SetTimestamp(in_img.GetTimestamp());

      this->out_image.Publish(img_out);

      this-> stop.Publish(stopDetect);
      this-> give_way.Publish(give_wayDetect);
      this-> cones.Publish(conesDetect);
      this-> right_of_way.Publish(right_of_wayDetect);
      this-> unimog.Publish(unimogDetect);

      this-> stop_size.Publish(stopSize);
      this-> give_way_size.Publish(give_waySize);
      this-> cones_size.Publish(conesSize);
      this-> right_of_way_size.Publish(right_of_waySize);
      this-> unimog_size.Publish(unimogSize);


    }
  }

}
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
