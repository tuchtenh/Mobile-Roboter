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
/*!\file    projects/rc_unimog_control_ex5/mImageConverter.cpp
 *
 * \author  Marcel Suiker
 *
 * \date    2020-11-27
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_ex5/mImageConverter.h"
#include <opencv2/opencv.hpp>
#include "rrlib/coviroa/opencv_utils.h"
#include "rrlib/color_blob_detection/cmvision.h"
#include <cstring>
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
using namespace cv;
//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace rc_unimog_control_ex5
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mImageConverter> cCREATE_ACTION_FOR_M_IMAGECONVERTER("ImageConverter");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mImageConverter constructor
//----------------------------------------------------------------------
mImageConverter::mImageConverter(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false), // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
  brightness(0)
{}

//----------------------------------------------------------------------
// mImageConverter destructor
//----------------------------------------------------------------------
mImageConverter::~mImageConverter()
{}

//----------------------------------------------------------------------
// mImageConverter OnStaticParameterChange
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// mImageConverter OnParameterChange
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mImageConverter Update
//----------------------------------------------------------------------
void mImageConverter::Update()
{
  if (this->in_image.HasChanged())
  {
    data_ports::tPortDataPointer<const std::vector<rrlib::coviroa::tImage>> in_img = this->in_image.GetPointer();
    if (in_img->size() > 0)
    {
      Mat image = rrlib::coviroa::AccessImageAsMat(in_img->at(0));
      data_ports::tPortDataPointer<rrlib::coviroa::tImage> img_out = this->out_image.GetUnusedBuffer();
      rrlib::rtti::GenericOperations<rrlib::coviroa::tImage>::DeepCopy((*in_img)[0], *img_out);
      cv::Mat out = rrlib::coviroa::AccessImageAsMat(*img_out);


      // Create and initialize the CMVision library
      CMVision* vision = new CMVision();
      vision->initialize(0, 0);      // again, assume 0x0 now and resize later
      String map_filename = "";
      String manual_map_filename = "color.txt";
      String color_filename = "cmvision_color_lut_combined.bin";
      bool use_custom_color_map(true);

      float den_merge = 0.35;
      int max_merge = 25;



      bool print_lut = false;

      if (use_custom_color_map)
      {
        // set up first class as orange
        if (color_filename == "")
        {
          vision->SetColorClass(0, 255, 0,   0, "First Class", den_merge, max_merge);
        }
        else
        {
          FILE * input;
          input = fopen(color_filename.c_str(), "r");
          if (input == NULL)
          {
            fprintf(stderr, "lut >> could not open '%s'!\n", color_filename.c_str());
            vision->SetColorClass(0, 255, 0,   0, "First Class", den_merge, max_merge);
          }
          else
          {
            int c1, c2, c3;
            char buffer[128];
            int cclass = 0;
            while (fscanf(input, "%d %d %d %s\n", &c1, &c2, &c3, (char*)(&buffer)) != EOF)
            {
              fprintf(stdout, "set color class named %s to %i %i %i\n", buffer, c1, c2, c3);
              vision->SetColorClass(cclass, c1, c2, c3, buffer, den_merge, max_merge);
              cclass++;
            }
            fclose(input);
          }
        }
        // load and set custom map
        if (vision->ImportClassMap(map_filename.c_str()) <= 0)
        {
          exit(1);
        }
      }
      else
      {
        if (vision->loadOptions(manual_map_filename.c_str()) == false)
        {
          exit(1);
        }

      }
      //
      // search database images
      //



      if (print_lut)
      {
        vision->PrintLookUpTable();
        vision->PrintCustomLookUpTable();
      }

      // Adapt blob detector to input image size, if needed
      if ((int) vision->width != image.cols || (int) vision->height != image.rows)
      {
        vision->initialize(image.cols, image.rows);
        vision->enable(CMV_COLOR_AVERAGES);
        vision->enable(CMV_DENSITY_MERGE);
        if (use_custom_color_map)
        {
          vision->enable(TUKL_CUSTOM_THRESHOLD_LUT);
        }


        vision->testClassify((unsigned char*) img_out.get(), (unsigned char*) in_img->data());
        vision->processFrame((unsigned char*) in_img->data());
        const CMVision::region* reg;

        reg = vision->getRegions(0);
        reg = vision->getRegions(0);
        for (int i = 0; i < vision->numRegions(0); i++)
        {
          //DrawRectangle(output_image, reg->x1, reg->y1, reg->x2, reg->y2, col, col, col);

          cv::rectangle(out, cvPoint(reg->x1, reg->y1), cvPoint(reg->x2, reg->y2), CV_RGB(128, 128, 128));
          reg = reg->next;
        }
        image.convertTo(out, -1, 1, brightness.Get());
        cv::rectangle(out,  Point(20, 20), Point(100, 100), Scalar(50, 250, 50));
        this->out_image.Publish(img_out);


      }
    }
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

