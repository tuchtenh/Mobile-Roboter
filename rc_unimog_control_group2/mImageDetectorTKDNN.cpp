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
     //DOES NOT WORK WITHOUT THOSE 2 LINES, BUT DOES NOT COMPILE WITH THEM
     //tk::dnn::Yolo3Detection yolo;
     //detNN = &yolo;
     
     int n_classes = 5;
     n_batch= 1;
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
        if (in_img->size() > 0){
            
            
            
            const cv::Mat cam_image_img = rrlib::coviroa::AccessImageAsMat(in_img->at(0));


            bool stopDetect = false, give_wayDetect = false, conesDetect = false, right_of_wayDetect = false, unimogDetect  = false;
           
                
            
            std::vector<cv::Mat> batch_frame;
            std::vector<cv::Mat> batch_dnn_input;
            
            
            batch_dnn_input.push_back(cam_image_img);
            batch_frame.push_back(cam_image_img.clone());
            
            detNN->update(batch_dnn_input, n_batch);
            detNN->draw(batch_frame);
            
             ///DETECT HERE
            detected_bbox.clear();
            detected_bbox = detNN->detected;
            for (auto d: detected_bbox)
            {
            
                switch(d.cl){
                    case 0: //stop
                        if(d.w * d.h > 4000){
                            stopDetect = true;
                        }
                        break;
                    case 1: 
                        if(d.w * d.h > 4000){
                            give_wayDetect = true;
                        }
                        break;
                    case 2:
                        if(d.w * d.h > 4000){
                           right_of_wayDetect = true;
                            
                        }
                        break;
                    case 3:
                        if(d.w * d.h > 4000){
                            conesDetect = true;
                        }
                        break;
                    case 4://unimog
                        if(d.w * d.h > 8000){
                            unimogDetect = true;
                        }
                        break;
                    }
                               
            }
            
            // prepare output image
            data_ports::tPortDataPointer<rrlib::coviroa::tImage> img_out = this->out_image.GetUnusedBuffer();
            // copy input image to output image
            rrlib::rtti::GenericOperations<rrlib::coviroa::tImage>::DeepCopy((*in_img)[0], *img_out);
            // use the tImage data acs an cv::Mat (does not copy the data)
            cv::Mat out = rrlib::coviroa::AccessImageAsMat(*img_out);
            
            img_out.SetTimestamp(in_img.GetTimestamp());
           
            this->out_image.Publish(img_out);
            
            this-> stop.Publish(stopDetect);
            this-> give_way.Publish(give_wayDetect);
            this-> cones.Publish(conesDetect);
            this-> right_of_way.Publish(right_of_wayDetect);
            this-> unimog.Publish(unimogDetect);

        }
    }
    
}
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
