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
/*!\file    projects/finroc_projects_robprak2020_2/mImageDetectorYolo.cpp
 *
 * \author  Manuel Vogel
 *
 * \date    2021-01-08
 *
 */
//----------------------------------------------------------------------
#include "projects/finroc_projects_robprak2020_2/mImageDetectorYolo.h"
#include <image.h>
//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "rrlib/coviroa/opencv_utils.h"

#ifdef __cplusplus
extern "C" {
#include "detection_layer.h"
#include "network.h"
#include "parser.h"
#include "region_layer.h"
#include "utils.h"
#endif
image mat_to_image(cv::Mat m);
cv::Mat image_to_mat(image im);
}

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
runtime_construction::tStandardCreateModuleAction<mImageDetectorYolo> cCREATE_ACTION_FOR_M_IMAGEDETECTORYOLO("ImageDetectorYolo");

//----------------------------------------------------------------------
// Implementation
//</----------------------------------------------------------------------

//----------------------------------------------------------------------
// mImageDetectorYolo constructor
//----------------------------------------------------------------------
mImageDetectorYolo::mImageDetectorYolo(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)

{
    RRLIB_LOG_PRINT(DEBUG, "Load alphabet...");
    this->alphabet_ = load_alphabet();
    RRLIB_LOG_PRINT(DEBUG, "Load network...");
    this->nw_ = load_network(const_cast<char*>(unimog_config.c_str()), const_cast<char*>(unimog_weights.c_str()), 0);
    RRLIB_LOG_PRINT(DEBUG, "Set layer ...");
    this->l_ = nw_->layers[this->nw_->n - 1];
    RRLIB_LOG_PRINT(DEBUG, "Set batch network...");
    //set_batch_network(this->nw_, 1);
    srand(2222222);
    nms_ = 0.4;
    RRLIB_LOG_PRINT(DEBUG, "Finished setup");
    //new object_detection::mbbDarknetYOLO(this);
}

//----------------------------------------------------------------------
// mImageDetectorYolo destructor
//----------------------------------------------------------------------
mImageDetectorYolo::~mImageDetectorYolo()
{}

//----------------------------------------------------------------------
// mImageDetectorYolo OnStaticParameterChange
//----------------------------------------------------------------------
void mImageDetectorYolo::OnStaticParameterChange()
{
  
}

//----------------------------------------------------------------------
// mImageDetectorYolo OnParameterChange
//----------------------------------------------------------------------
void mImageDetectorYolo::OnParameterChange()
{
 
}

//----------------------------------------------------------------------
// mImageDetectorYolo Update
//----------------------------------------------------------------------
void mImageDetectorYolo::Update()
{
    if (this->in_images.HasChanged())
    {
        data_ports::tPortDataPointer<const std::vector<rrlib::coviroa::tImage>> in_img = this->in_images.GetPointer();
        // check if the vector really contains at least one image
        if (in_img->size() > 0){
            // use the tImage data as an cv::Mat (does not copy the data)
            //const cv::Mat cam_image_bgr = rrlib::coviroa::AccessImageAsMat(in_img->at(0));
            //cv::Mat cam_image;
            //cv::cvtColor(rrlib::coviroa::AccessImageAsMat(in_img->at(0)), cam_image,cv::COLOR_BGRA2RGBA );
            cv::Mat cam_image = rrlib::coviroa::ConvertFormat<cv::Vec3b>(in_img->at(0), rrlib::coviroa::tImageFormat::eIMAGE_FORMAT_BGR24);

            // prepare output image
            data_ports::tPortDataPointer<rrlib::coviroa::tImage> img_out = this->out_image.GetUnusedBuffer();
            // copy input image to output image
            rrlib::rtti::GenericOperations<rrlib::coviroa::tImage>::DeepCopy((*in_img)[0], *img_out);
            // use the tImage data acs an cv::Mat (does not copy the data)
            cv::Mat out;// = rrlib::coviroa::AccessImageAsMat(*img_out);
            cv::cvtColor(rrlib::coviroa::AccessImageAsMat(*img_out), out,cv::COLOR_BGRA2RGBA );

            //cv::imshow("test",cam_image);



            //detect objects
            image im = mat_to_image(cam_image);
            image sized = resize_image(im, this->nw_->w, this->nw_->h);

            float *data_ptr = sized.data;

            // run classifier
            network_predict(*nw_, data_ptr);
            //network_predict_image(this->nw_, sized);

            // get result
            int nboxes = 0;
            //detection *dets = make_network_boxes(this->nw_,threshold,&nboxes);
            //detection *dets = get_network_boxes(this->nw_, this->nw_->w, this->nw_->h, threshold, 0,0, , &nboxes);
            dets = get_network_boxes(this->nw_, cam_image.cols, cam_image.rows, threshold, 0 ,0, 0, &nboxes,0);
            if (nms_)
            {
              do_nms_sort(dets, l_.side * l_.side * l_.n, l_.classes, nms_);
            }
            std::cout<<"image size"<<this->nw_->h<<" x " <<this->nw_->w<<std::endl;
          
/*
            // copy result to output data structure
            std::vector<tDarknetDetection> detections;
            for (int i = 0; i < nboxes; i++)
            {
              rrlib::geometry::tBoundingBox<2, float> bounding_box
              {
                rrlib::geometry::tPoint<2, float>(
                  (dets[i].bbox.x - dets[i].bbox.w / 2.0f) * static_cast<float>(cam_image.cols),
                  (dets[i].bbox.y - dets[i].bbox.h / 2.0f) * static_cast<float>(cam_image.cols)) ,
                rrlib::geometry::tPoint<2, float>(
                  (dets[i].bbox.x + dets[i].bbox.w / 2.0f) * static_cast<float>(cam_image.cols),
                  (dets[i].bbox.y + dets[i].bbox.h / 2.0f) * static_cast<float>(cam_image.rows))
              };
              tDarknetUnimogClass unimog_class;
              float probability = 0.0f;

              for (size_t j = 0; j < classCount; j++)
              {
                probability = dets[i].prob[j];
                if (probability >= threshold)
                {
                  unimog_class =  static_cast<tDarknetUnimogClass>(j);
                  detections.emplace_back(tDarknetDetection(bounding_box, unimog_class, probability));
                }
              }
            }
            */

            //get the data
            for (int i = 0; i < nboxes; i++)
            {
                  rrlib::geometry::tPoint<2, float> point (
                  dets[i].bbox.x * static_cast<float>(cam_image.cols),
                  dets[i].bbox.y * static_cast<float>(cam_image.rows));
                  tDarknetUnimogClass unimog_class;

              float probability = 0.0f;
              for (size_t j = 0; j < classCount; j++)
              {
                //std::cout<<dets[i].prob[j]<<std::endl;
                probability = dets[i].prob[j];
                //std::cout<<"found something with class (bad prob): "<<j<<"prob:"<<probability<<std::endl;
                if (probability >= threshold)
                { 
                  switch(j){
                      case(0): 
                          
                  }
                      
                  std::cout<<"found something with class: "<<j<<std::endl;
                  unimog_class =  static_cast<tDarknetUnimogClass>(j);
                }
              }
            }

            const char * class_NAMES[] =
            {
            "stop",
            "sign1",
            "cones",
            "sign2",
            "unimog",
            };


            // visualize
            draw_detections_v3(im, dets, nboxes, threshold, const_cast<char**>(class_NAMES), alphabet_, 5,1);
            image_to_mat(im).copyTo(out);


            // pop up window in case of result
            //save_image(im, "detections");
            //show_image(im, "detections", 0);

            // clean up
            free_detections(dets, nboxes);
            free_image(im);
            free_image(sized);



            this->out_image.Publish(img_out);

        }
    }
    
}
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
