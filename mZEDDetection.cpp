// 11 Feb 2021
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
/*!\file    projects/rc_unimog_control_ub5/mZEDDetection.cpp
 *
 * \author  Lukas Tuchtenhagen
 *
 * \date    2020-12-03
 *
 */
//----------------------------------------------------------------------
#include "projects/finroc_projects_robprak2020_2/mZEDDetection.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include <unistd.h>
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
runtime_construction::tStandardCreateModuleAction<mZEDDetection> cCREATE_ACTION_FOR_M_ZEDDETECTION("ZEDDetection");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mZEDDetection constructor
//----------------------------------------------------------------------
mZEDDetection::mZEDDetection(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
{}

//----------------------------------------------------------------------
// mZEDDetection destructor
//----------------------------------------------------------------------
mZEDDetection::~mZEDDetection()
{}


//----------------------------------------------------------------------
// mZEDDetection OnParameterChange
//----------------------------------------------------------------------
void mZEDDetection::OnParameterChange()
{
}

//----------------------------------------------------------------------
// mZEDDetection Update
//----------------------------------------------------------------------
void mZEDDetection::Update()
{
  // check for new images
  if (this->camera_in.HasChanged())
  {

    data_ports::tPortDataPointer<const std::vector<rrlib::coviroa::tImage>> in_img = this->camera_in.GetPointer();
    // check if the vector really contains a least one image
    if (in_img->size() > 0)
    {

      cv::Mat img = rrlib::coviroa::AccessImageAsMat(in_img->at(0));
      // prepare output image
      data_ports::tPortDataPointer<rrlib::coviroa::tImage> img_out = this->camera_out.GetUnusedBuffer();
      // copy input image to output image
      rrlib::rtti::GenericOperations<rrlib::coviroa::tImage>::DeepCopy((*in_img)[0], *img_out);
      // use the tImage data acs an cv::Mat (does not copy the data)




      //___________________________________________________________________
      // Detection of red side lines

      cv::Mat out = rrlib::coviroa::AccessImageAsMat(*img_out);
      cv::Mat outred = rrlib::coviroa::AccessImageAsMat(*img_out);
      // apply gaussian blur
      // img.convertTo(out,-1,1,50);
      /*if ((old_left > 500 && old_right < -500) || (old_left < -500 || old_right > 500))
            {
              if (contrast_change)
              {
                contrast -= 0.25;
                if (contrast > 2)
                {
                  contrast_change = false;
                }
              }
              if (!contrast_change)
              {
                contrast += 0.25;
                if (contrast < 0.5)
                {
                  contrast_change = true;
                }
              }
            }*/

      cv::cvtColor(outred, outred, cv::COLOR_BGR2HSV);
      cv::Mat hsvchannel[3], saturated;
      cv::split(outred, hsvchannel);
      cv::Mat binaryred;

      /*
      hsvchannel[1] = hsvchannel[1] * 4.5;      // 2
      std::vector<cv::Mat> hsv = {hsvchannel[0], hsvchannel[1], hsvchannel[2]};
      cv::merge(hsv, saturated);
      cv::cvtColor(saturated, outred, cv::COLOR_HSV2BGR);
      // TODO cv::imwrite("bildSATURATED.png", outred);
      outred.convertTo(outred, -1, contrast, 0);       // 2
      // TODO cv::imwrite("bildContrast.png", outred);
      //cv::Mat binaryred;

      cv::GaussianBlur(outred, outred, cv::Size(5, 5), 0);

      //inverted
      cv::bitwise_not(outred, outred);
      //cv::imwrite("inverted.png", outred);

      cv::cvtColor(outred, outred, cv::COLOR_BGR2HSV);
      // range red
      // cv::inRange(outred, cv::Scalar(1, 200, 200), cv::Scalar(15, 255, 255), binaryred);
      // range with inverted
      cv::inRange(outred, cv::Scalar(85, 120, 230), cv::Scalar(95, 255, 255), binaryred);
      // TODO cv::imwrite("bildMaskeRot.png", binaryred);
        */

      if (colorSwitchEnable.Get() == true)
      {
        if (blueOrRed.Get() == 0)
        {
          colorState = BLUE;
        }

        else
        {
          colorState = RED;
        }
      }

      else
      {
        if (colorSwitchFromEasy.Get() == 0)
        {
          colorState = BLUE;
        }

        else
        {
          colorState = RED;
        }
      }


      if (colorState == BLUE)
      {
        hsvchannel[1] = hsvchannel[1] * 4.5;      // 2
        std::vector<cv::Mat> hsv = {hsvchannel[0], hsvchannel[1], hsvchannel[2]};
        cv::merge(hsv, saturated);
        cv::cvtColor(saturated, outred, cv::COLOR_HSV2BGR);
        // TODO cv::imwrite("bildSATURATED.png", outred);
        outred.convertTo(outred, -1, contrast, 0);       // 2
        // TODO cv::imwrite("bildContrast.png", outred);
        //cv::Mat binaryred;

        cv::GaussianBlur(outred, outred, cv::Size(5, 5), 0);

        //inverted
        cv::bitwise_not(outred, outred);
        //cv::imwrite("inverted.png", outred);

        cv::cvtColor(outred, outred, cv::COLOR_BGR2HSV);
        // range red
        // cv::inRange(outred, cv::Scalar(1, 200, 200), cv::Scalar(15, 255, 255), binaryred);
        // range with inverted
        cv::inRange(outred, cv::Scalar(85, 120, 230), cv::Scalar(95, 255, 255), binaryred);
        // TODO cv::imwrite("bildMaskeRot.png", binaryred);

        gui_colorSwitch.Publish(false);
      }
      else
      {
        hsvchannel[1] = hsvchannel[1] * 5;      // 2
        std::vector<cv::Mat> hsv = {hsvchannel[0], hsvchannel[1], hsvchannel[2]};
        cv::merge(hsv, saturated);
        cv::cvtColor(saturated, outred, cv::COLOR_HSV2BGR);
        // TODO cv::imwrite("bildSATURATED.png", outred);
        outred.convertTo(outred, -1, contrast, 0);       // 2
        // TODO cv::imwrite("bildContrast.png", outred);
        //cv::Mat binaryred;

        cv::GaussianBlur(outred, outred, cv::Size(5, 5), 0);

        //cv::imwrite("inverted.png", outred);

        cv::cvtColor(outred, outred, cv::COLOR_BGR2HSV);
        // range red
        cv::inRange(outred, cv::Scalar(1, 200, 200), cv::Scalar(15, 255, 255), binaryred);
        // TODO cv::imwrite("bildMaskeRot.png", binaryred);

        gui_colorSwitch.Publish(true);
      }



      cv::cvtColor(outred, outred, cv::COLOR_HSV2BGR);

      cv::cvtColor(outred, outred, cv::COLOR_BGR2GRAY);
      cv::threshold(outred, outred, 128, 255, cv::THRESH_BINARY);

      //masking the image white
      cv::Mat maskred = cv::Mat::zeros(outred.size(), outred.type());
      cv::Point ptsred[4] =
      {
        /*
          cv::Point(0, 270), // links oben
          cv::Point(672, 270), // rechts oben
          //cv::Point(490, 340), // rechts oben
          //cv::Point(520, 270), // rechts oben RC_UNIMOG
          //cv::Point(520, 376), //rechts unten RC_UNIMOG
          cv::Point(672, 376), //rechts unten
          //cv::Point(605, 376), //rechts unten
          cv::Point(0, 376) //links unten
        */

        cv::Point(left_red_x, left_red_y), // 270 links oben
        cv::Point(right_red_x, right_red_y), // 270 rechts oben
        cv::Point(right_red_x, 376), //rechts unten
        cv::Point(left_red_x, 376) //links unten


      };

      // Create a binary polygon mask
      cv::fillConvexPoly(maskred, ptsred, 4, cv::Scalar(255, 0, 0));
      // Multiply the edges image and the mask to get the output
      cv::bitwise_and(binaryred, maskred, outred);

      // TODO cv::imwrite("bildMASKred.png", outred);

      std::vector<cv::Vec4i> linesred; // will hold the results of the detection
      cv::HoughLinesP(outred, linesred, 1, CV_PI / 180, 1, 0, 0); // runs the actual detection       //(1) 20 instead of 50     @@@@@@@@@@@@@@@@@@@@@

      //___________________________________________________________________
      // Detection of white middle lane

      cv::GaussianBlur(out, out, cv::Size(5, 5), 0);
      // turn the image to grayscale
      cv::cvtColor(out, out, cv::COLOR_BGR2GRAY);
      //cv::imwrite("bildGRAY.png", out);

      cv::threshold(out, out, gray_treshold, 255, cv::THRESH_BINARY); // 128
      // TODO cv::imwrite("bildBINRAY.png", out);

      //masking the image white
      cv::Mat mask = cv::Mat::zeros(out.size(), out.type());

      /*
            a = m * (M + 70);

            if( a<=0 || a >400)
            {
              a = 0;
            }


            std::cout << "a: " << a << std::endl;
      */
      a = 0;

      cv::Point pts[4] =
      {

        cv::Point(left_white_x + a, left_white_y), // links oben
        cv::Point(right_white_x + a, right_white_y), // rechts oben
        cv::Point(right_white_x + a, 376), //rechts unten
        cv::Point(left_white_x + a, 376) //links unten
        /*
          cv::Point(135, 330), // links oben
          cv::Point(340, 330), // rechts oben
          //cv::Point(490, 340), // rechts oben
          //cv::Point(520, 270), // rechts oben RC_UNIMOG
          //cv::Point(520, 376), //rechts unten RC_UNIMOG
          cv::Point(340, 376), //rechts unten
          //cv::Point(605, 376), //rechts unten
          cv::Point(135, 376) //links unten
          */

        /*
         cv::Point(left_white_x, left_white_y), // links oben
         cv::Point(right_white_x, right_white_y), // rechts oben
         cv::Point(right_white_x, 376), //rechts unten
         cv::Point(left_white_x, 376) //links unten
        */






      };

      // Create a binary polygon mask
      cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 0, 0));
      // Multiply the edges image and the mask to get the output
      cv::bitwise_and(out, mask, out);

      //TODO cv::imwrite("bildMASK.png", out);

      // calculating HoughLines
      std::vector<cv::Vec4i> lines; // will hold the results of the detection
      cv::HoughLinesP(out, lines, 1, CV_PI / 180, 25, 0, 0); // runs the actual detection


      //___________________________________________________________________

      // line categorization
      std::vector<cv::Vec4i> right_lines, left_lines, mid_lines;
      std::vector<cv::Point> left_pts, right_pts, mid_pts;
      cv::Vec4d right_line, left_line, mid_line;
      //double left, right;
      //left = 250; //230    // 280
      //right = 340;    // 430
      cv::Point right_b;
      double right_m;
      cv::Point left_b;
      double left_m;
      cv::Point mid_b;
      double mid_m;
      cv::Point ini;
      cv::Point fini;
      double ini_y = 376;
      double fini_y = 270;


      // categorize side lines
      for (size_t i = 0; i < linesred.size(); i++)
      {
        cv::Vec4i l = linesred[i];
        if (l[2] < red_detection_threshold)  // if (l[2] < left)
        {
          left_lines.push_back(l);
        }
        else if (red_detection_threshold < l[2]) // else if (right < l[2])
        {
          right_lines.push_back(l);
        }
        cv::line(img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 255, 255, 255), 5, CV_AA);
      }


      // categorize mid lines
      for (size_t i = 0; i < lines.size(); i++)
      {

        cv::Vec4i l = lines[i];
        if (l[2] > left_white_x && right_white_x > l[2]) // if (l[2] > 170 && 380 > l[2])
        {
          mid_lines.push_back(l);
        }
        cv::line(img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 255, 0, 255), 5, CV_AA);
      }


      // TODO cv::imwrite("bildHOUGHLINES.png", img);

      //std::cout << lines.size() << " mid lines" << std::endl;
      //std::cout << linesred.size() << " side lines" << std::endl;
      // start and end of the lines stored as points
      for (auto i : left_lines)
      {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);

        left_pts.push_back(ini);
        left_pts.push_back(fini);
      }

      if (left_pts.size() > 0)
      {
        // The left line is formed here

        cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
        left_m = left_line[1] / left_line[0];
        left_b = cv::Point(left_line[2], left_line[3]);
      }
      // start and end of the lines stored as points
      ////std::cout<<mid_lines.size()<<std::endl;
      for (auto i : mid_lines)
      {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);
        //std::cout<< "point "<<i[0]<<" "<<i[1]<<" "<<i[2]<<" "<<i[3]<<std::endl;
        mid_pts.push_back(ini);
        mid_pts.push_back(fini);
      }

      if (mid_pts.size() > 0)
      {
        // The middle line is formed here

        cv::fitLine(mid_pts, mid_line, CV_DIST_L2, 0, 0.01, 0.01);

        mid_m = mid_line[1] / mid_line[0];
        mid_b = cv::Point(mid_line[2], mid_line[3]);
      }
      // start and end of the lines stored as points
      for (auto i : right_lines)
      {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);

        right_pts.push_back(ini);
        right_pts.push_back(fini);
      }

      if (right_pts.size() > 0)
      {
        // The right line is formed here
        cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
        right_m = right_line[1] / right_line[0];
        right_b = cv::Point(right_line[2], right_line[3]);
      }


      // calculate start and finish points for right, mid and left lines
      double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
      double right_fini_x = ((fini_y - right_b.y) / right_m) + right_b.x;
      //std::cout << right_ini_x << " ";
      //std::cout << right_fini_x << " ";
      //std::cout << right_m << std::endl;

      double mid_ini_x = ((ini_y - mid_b.y) / mid_m) + mid_b.x;
      double mid_fini_x = ((fini_y - mid_b.y) / mid_m) + mid_b.x;

      //std::cout << mid_ini_x << " " ;
      //std::cout << mid_fini_x << " " ;
      //std::cout << mid_m << std::endl;

      double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
      double left_fini_x = ((fini_y - left_b.y) / left_m) + left_b.x;

      //*/

      /*
      double ini_y = 376;
      double fini_y = 240;
      std::vector<double> mid = MidDetection();
      std::vector<double> sides = SideDetection();
      double left_ini_x = sides[0];
      double left_fini_x = sides[1];
      double right_ini_x = sides[2];
      double right_fini_x = sides[3];
      double mid_ini_x = mid[0];
      double mid_fini_x = mid[1];
      */

      //  draw the lines
      cv::line(img, cv::Point(left_ini_x, ini_y), cv::Point(left_fini_x, fini_y), cv::Scalar(0, 0, 255, 255), 5, CV_AA);
      cv::line(img, cv::Point(mid_ini_x, ini_y), cv::Point(mid_fini_x, fini_y), cv::Scalar(0, 255, 0, 255), 5, CV_AA);
      cv::line(img, cv::Point(right_ini_x, ini_y), cv::Point(right_fini_x, fini_y), cv::Scalar(255, 0, 0, 255), 5, CV_AA);

      //cv::imwrite("bildFittedLines.png", img);

      double midpixel = img.cols / 2;
      old_left = left_ini_x - midpixel;
      old_mid = mid_ini_x - midpixel;
      old_right = right_ini_x - midpixel;

      if (queueLeft.size() < 999 && old_left != std::numeric_limits<double>::infinity())
      {
        queueLeft.push(old_left);
      }
      else if (old_left != std::numeric_limits<double>::infinity())
      {
        queueLeft.push(old_left);
        this->angle_to_left_out.Publish(std::acos((queueLeft.back() - queueLeft.front()) / (std::sqrt(std::pow(queueLeft.back() - queueLeft.front(), 2) + 100))));
        queueLeft.pop();
      }

      if (queueMid.size() < 999 && old_mid != std::numeric_limits<double>::infinity())
      {
        queueMid.push(old_mid);
      }
      else if (old_mid != std::numeric_limits<double>::infinity())
      {
        queueMid.push(old_mid);
        this->angle_to_mid_out.Publish(std::acos((queueMid.back() - queueMid.front()) / (std::sqrt(std::pow(queueMid.back() - queueMid.front(), 2) + 100))));
        queueMid.pop();
      }
      if (queueRight.size() < 999 && old_right != std::numeric_limits<double>::infinity())
      {
        queueRight.push(old_right);
      }
      else if (old_right != std::numeric_limits<double>::infinity())
      {
        queueRight.push(old_right);
        this->angle_to_right_out.Publish(std::acos((queueRight.back() - queueRight.front()) / (std::sqrt(std::pow(queueRight.back() - queueRight.front(), 2) + 100))));
        queueRight.pop();
      }

      this->distance_to_left_out.Publish(old_left);
      this->distance_to_mid_out.Publish(old_mid);
      this->distance_to_right_out.Publish(old_right);


      //M = old_mid;

      // Make one image
      //usleep(100000);


      //this->camera_out.Publish(img_out);

    }
  }

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
