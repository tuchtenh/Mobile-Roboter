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
/*!\file    projects/finroc_projects_robprak2020_2/rc_unimog_control_group2/mZEDDetection.cpp
 *
 * \author  Lukas Tuchtenhagen
 *
 * \date    2020-12-07
 *
 */
//----------------------------------------------------------------------
#include "projects/finroc_projects_robprak2020_2/rc_unimog_control_group2/mZEDDetection.h"

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
namespace rc_unimog_control_group2
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
// mZEDDetection OnStaticParameterChange
//----------------------------------------------------------------------
std::vector<double> mZEDDetection::MidDetection()
{
  std::vector<double> result;
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
    cv::Mat out = rrlib::coviroa::AccessImageAsMat(*img_out);
    // apply gaussian blur
    // img.convertTo(out,-1,1,50);
    cv::GaussianBlur(out, out, cv::Size(5, 5), 0);
    // turn the image to grayscale
    cv::cvtColor(out, out, cv::COLOR_BGR2GRAY);
    cv::imwrite("bildGRAY.png", out);

    cv::threshold(out, out, 128, 255, cv::THRESH_BINARY);
    cv::imwrite("bildBINRAY.png", out);

    //masking the image
    cv::Mat mask = cv::Mat::zeros(out.size(), out.type());
    cv::Point pts[4] =
    {
      cv::Point(0, 240), // links oben
      cv::Point(672, 240), // rechts oben
      //cv::Point(490, 340), // rechts oben
      cv::Point(672, 376), //rechts unten
      //cv::Point(605, 376), //rechts unten
      cv::Point(0, 376) //links unten

    };

    // Create a binary polygon mask
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 0, 0));
    // Multiply the edges image and the mask to get the output
    cv::bitwise_and(out, mask, out);
    cv::imwrite("bildMASK.png", out);

    // calculating HoughLines
    std::vector<cv::Vec4i> lines; // will hold the results of the detection
    cv::HoughLinesP(out, lines, 1, CV_PI / 180, 50, 0, 0); // runs the actual detection

    std::vector<cv::Vec4i>  mid_lines;
    std::vector<cv::Point>  mid_pts;
    cv::Vec4d mid_line;
    double left, right;
    left = 280;
    right = 430;
    cv::Point mid_b;
    double mid_m;
    cv::Point ini;
    cv::Point fini;
    double ini_y = 376;
    double fini_y = 240;

    // categorize the lines
    for (size_t i = 0; i < lines.size(); i++)
    {

      cv::Vec4i l = lines[i];
      if (l[2] > left && right > l[2])
      {
        mid_lines.push_back(l);
      }
      //cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255,255), 5, CV_AA);
    }

    cv::imwrite("bildHOUGHLINES.png", img);


    // start and end of the lines stored as points
    for (auto i : mid_lines)
    {
      ini = cv::Point(i[0], i[1]);
      fini = cv::Point(i[2], i[3]);

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

    double mid_ini_x = ((ini_y - mid_b.y) / mid_m) + mid_b.x;
    double mid_fini_x = ((fini_y - mid_b.y) / mid_m) + mid_b.x;
    result.push_back(mid_ini_x);
    result.push_back(mid_fini_x);




  }
  return result;
}

std::vector<double> mZEDDetection::SideDetection()
{
  std::vector<double> result;
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
    cv::Mat out = rrlib::coviroa::AccessImageAsMat(*img_out);
    // apply gaussian blur
    // img.convertTo(out,-1,1,50);
    cv::GaussianBlur(out, out, cv::Size(5, 5), 0);
    // turn the image to grayscale
    cv::cvtColor(out, out, cv::COLOR_BGR2GRAY);
    cv::imwrite("bildGRAY.png", out);

    cv::threshold(out, out, 128, 255, cv::THRESH_BINARY);
    cv::imwrite("bildBINRAY.png", out);

    //masking the image
    cv::Mat mask = cv::Mat::zeros(out.size(), out.type());
    cv::Point pts[4] =
    {
      cv::Point(0, 240), // links oben
      cv::Point(672, 240), // rechts oben
      //cv::Point(490, 340), // rechts oben
      cv::Point(672, 376), //rechts unten
      //cv::Point(605, 376), //rechts unten
      cv::Point(0, 376) //links unten

    };

    // Create a binary polygon mask
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 0, 0));
    // Multiply the edges image and the mask to get the output
    cv::bitwise_and(out, mask, out);
    cv::imwrite("bildMASK.png", out);

    // calculating HoughLines
    std::vector<cv::Vec4i> lines; // will hold the results of the detection
    cv::HoughLinesP(out, lines, 1, CV_PI / 180, 50, 0, 0); // runs the actual detection

    std::vector<cv::Vec4i> right_lines, left_lines;
    std::vector<cv::Point> left_pts, right_pts;
    cv::Vec4d right_line, left_line;
    double left, right;
    left = 280;
    right = 430;
    cv::Point right_b;
    double right_m;
    cv::Point left_b;
    double left_m;
    cv::Point ini;
    cv::Point fini;
    double ini_y = 376;
    double fini_y = 240;

    // categorize the lines
    for (size_t i = 0; i < lines.size(); i++)
    {

      cv::Vec4i l = lines[i];
      if (l[2] < left)
      {
        left_lines.push_back(l);
      }
      else if (right < l[2])
      {
        right_lines.push_back(l);
      }
      //cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255,255), 5, CV_AA);
    }

    cv::imwrite("bildHOUGHLINES.png", img);

    std::cout << lines.size() << std::endl;
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


    double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
    double right_fini_x = ((fini_y - right_b.y) / right_m) + right_b.x;

    double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
    double left_fini_x = ((fini_y - left_b.y) / left_m) + left_b.x;
    std::vector<double> result;
    result.push_back(left_ini_x);
    result.push_back(left_fini_x);
    result.push_back(right_ini_x);
    result.push_back(right_fini_x);



  }
  return result;
}

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
      cv::Mat out = rrlib::coviroa::AccessImageAsMat(*img_out);
      // apply gaussian blur
      // img.convertTo(out,-1,1,50);
      cv::GaussianBlur(out, out, cv::Size(5, 5), 0);
      // turn the image to grayscale
      cv::cvtColor(out, out, cv::COLOR_BGR2GRAY);
      cv::imwrite("bildGRAY.png", out);

      cv::threshold(out, out, 128, 255, cv::THRESH_BINARY);
      cv::imwrite("bildBINRAY.png", out);

      //masking the image
      cv::Mat mask = cv::Mat::zeros(out.size(), out.type());
      cv::Point pts[4] =
      {
        cv::Point(0, 240), // links oben
        cv::Point(672, 240), // rechts oben
        //cv::Point(490, 340), // rechts oben
        cv::Point(672, 376), //rechts unten
        //cv::Point(605, 376), //rechts unten
        cv::Point(0, 376) //links unten

      };

      // Create a binary polygon mask
      cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 0, 0));
      // Multiply the edges image and the mask to get the output
      cv::bitwise_and(out, mask, out);
      cv::imwrite("bildMASK.png", out);

      // calculating HoughLines
      std::vector<cv::Vec4i> lines; // will hold the results of the detection
      cv::HoughLinesP(out, lines, 1, CV_PI / 180, 50, 0, 0); // runs the actual detection

      std::vector<cv::Vec4i> right_lines, left_lines, mid_lines;
      std::vector<cv::Point> left_pts, right_pts, mid_pts;
      cv::Vec4d right_line, left_line, mid_line;
      double left, right;
      left = 280;
      right = 430;
      cv::Point right_b;
      double right_m;
      cv::Point left_b;
      double left_m;
      cv::Point mid_b;
      double mid_m;
      cv::Point ini;
      cv::Point fini;
      double ini_y = 376;
      double fini_y = 240;

      // categorize the lines
      for (size_t i = 0; i < lines.size(); i++)
      {

        cv::Vec4i l = lines[i];
        if (l[2] < left)
        {
          left_lines.push_back(l);
        }
        else if (right < l[2])
        {
          right_lines.push_back(l);
        }
        else
        {
          mid_lines.push_back(l);
        }
        //cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255,255), 5, CV_AA);
      }

      cv::imwrite("bildHOUGHLINES.png", img);

      std::cout << lines.size() << std::endl;
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
      for (auto i : mid_lines)
      {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);

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


      double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
      double right_fini_x = ((fini_y - right_b.y) / right_m) + right_b.x;
      std::cout << right_ini_x << " ";
      std::cout << right_fini_x << std::endl;

      double mid_ini_x = ((ini_y - mid_b.y) / mid_m) + mid_b.x;
      double mid_fini_x = ((fini_y - mid_b.y) / mid_m) + mid_b.x;

      std::cout << mid_ini_x << " " ;
      std::cout << mid_fini_x << std::endl;

      double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
      double left_fini_x = ((fini_y - left_b.y) / left_m) + left_b.x;

      cv::line(img, cv::Point(left_ini_x, ini_y), cv::Point(left_fini_x, fini_y), cv::Scalar(0, 0, 255, 255), 5, CV_AA);
      cv::line(img, cv::Point(mid_ini_x, ini_y), cv::Point(mid_fini_x, fini_y), cv::Scalar(0, 255, 0, 255), 5, CV_AA);
      cv::line(img, cv::Point(right_ini_x, ini_y), cv::Point(right_fini_x, fini_y), cv::Scalar(255, 0, 0, 255), 5, CV_AA);

      cv::imwrite("bildFittedLines.png", img);

      double midpixel = img.cols / 2;

      if (queueLeft.size() < 9 && left_ini_x - midpixel != std::numeric_limits<double>::infinity())
      {
        queueLeft.push(left_ini_x - midpixel);
      }
      else if (left_ini_x - midpixel != std::numeric_limits<double>::infinity())
      {
        queueLeft.push(left_ini_x - midpixel);
        this->angle_to_left_out.Publish(std::acos((queueLeft.back() - queueLeft.front()) / (std::sqrt(std::pow(queueLeft.back() - queueLeft.front(), 2) + 100))));
        queueLeft.pop();
      }

      if (queueMid.size() < 9 && mid_ini_x - midpixel != std::numeric_limits<double>::infinity())
      {
        queueMid.push(mid_ini_x - midpixel);
      }
      else if (mid_ini_x - midpixel != std::numeric_limits<double>::infinity())
      {
        queueMid.push(mid_ini_x - midpixel);
        this->angle_to_mid_out.Publish(std::acos((queueMid.back() - queueMid.front()) / (std::sqrt(std::pow(queueMid.back() - queueMid.front(), 2) + 100))));
        queueMid.pop();
      }
      if (queueRight.size() < 9 && right_ini_x - midpixel != std::numeric_limits<double>::infinity())
      {
        queueRight.push(right_ini_x - midpixel);
      }
      else if (right_ini_x - midpixel != std::numeric_limits<double>::infinity())
      {
        queueRight.push(right_ini_x - midpixel);
        this->angle_to_right_out.Publish(std::acos((queueRight.back() - queueRight.front()) / (std::sqrt(std::pow(queueRight.back() - queueRight.front(), 2) + 100))));
        queueRight.pop();
      }

      this->distance_to_left_out.Publish((left_ini_x - midpixel));
      this->distance_to_mid_out.Publish((mid_ini_x - midpixel));
      this->distance_to_right_out.Publish((right_ini_x - midpixel));


      // Make one image
      usleep(10000);


      this->camera_out.Publish(img_out);

    }
  }

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
