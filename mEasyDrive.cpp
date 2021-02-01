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
/*!\file    projects/finroc_projects_robprak2020_2/mEasyDrive.cpp
 *
 * \author  Aaron Hackenberg
 *
 * \date    2020-12-03
 *
 */
//----------------------------------------------------------------------
#include "projects/finroc_projects_robprak2020_2/mEasyDrive.h"

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
#include <cmath>
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
runtime_construction::tStandardCreateModuleAction<mEasyDrive> cCREATE_ACTION_FOR_M_EASYDRIVE(
  "EasyDrive");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mEasyDrive constructor
//----------------------------------------------------------------------
mEasyDrive::mEasyDrive(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false), velocity(0), curvature(0), lock(false)
{


}

//----------------------------------------------------------------------
// mEasyDrive destructor
//----------------------------------------------------------------------
mEasyDrive::~mEasyDrive()
{
}

//----------------------------------------------------------------------
// mEasyDrive OnStaticParameterChange
//----------------------------------------------------------------------
void mEasyDrive::OnStaticParameterChange()
{
  /*if (this->static_parameter_1.HasChanged())
   {
   As this static parameter has changed, do something with its value!
   }*/
}

//----------------------------------------------------------------------
// mEasyDrive OnParameterChange
//----------------------------------------------------------------------
void mEasyDrive::OnParameterChange()
{
  //If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
}

//----------------------------------------------------------------------
// mEasyDrive Update
//----------------------------------------------------------------------
void mEasyDrive::Update()
{

  // choose which line to follow
  /*
    lineDet.setLineValue(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
    lineDet.chooseLine();
    int distance = lineDet.publishDistance();
    double pixel = lineDet.publishPixel();
  */
  if (test.Get() == true)
  {
    intersectDetPtr->interProcessOn = true;
  }


  std::tuple<int, double, bool> lineData(0, 0, false);

  if (intersectDetPtr->interProcessOn == true)
  {
    lineData = intersectDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
  }

  else
  {
    lineData = lineDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
  }


  int distance = std::get<0>(lineData);
  double pixel = std::get<1>(lineData);
  bool noDetect = std::get<2>(lineData);


  if (noDetect == true) //three lines detection fail
  {
    curvature = 0;
    std::cout << "Line distance is infinite." << std::endl;
  }

  else
  {
    expAlgrithm(distance, pixel);
    //velocity = 0.8;
  }



  //this->out_velocity.Publish(velocity);
  this->out_curvature.Publish(curvature);
  this->out_noLineDetection.Publish(noDetect);


}

//----------------------------------------------------------------------
// mEasyDrive drive_straight
// Publishes the velocity and curvature necessary to drive
// in a straight line within a lane
//----------------------------------------------------------------------
/*
void mEasyDrive::drive_straight()
{
  velocity = 1;
  curvature = 0;
  if (lock_checker < unlock_value)
  {
    lock_checker++;
  }
  else
  {
    lock = false;
  }
  //this->out_velocity.Publish(velocity);
  this->out_curvature.Publish(curvature);
  //this->out_velocity.Publish(1);
  //this->out_velocity.Publish(0);

}*/
//----------------------------------------------------------------------
// mEasyDrive drive_curve_1
// Publishes the velocity and curvature necessary to drive
// in a curve within a lane
//----------------------------------------------------------------------
/*void mEasyDrive::drive_curve_1() {
  velocity = input_velocity_1.Get();
  if (lock_checker < input_time_1.Get()) {
    curvature = input_curvature_1.Get();
  } else if (lock_checker < input_time_2.Get()) {
    curvature = input_curvature_2.Get();
  } else {
    curvature = input_curvature_3.Get();
  }
  if (lock_checker < unlock_value) {
    lock_checker++;
  } else {
    lock = false;
  }
  this->out_velocity.Publish(velocity);
  this->out_curvature.Publish(curvature);
  //this->out_velocity.Publish(1);
  //this->out_curvature.Publish(1.3);
}*/
//----------------------------------------------------------------------
// mEasyDrive drive_curve_2
// Publishes the inverse velocity and curvature necessary to drive
// in a curve within a lane
//----------------------------------------------------------------------
/*void mEasyDrive::drive_curve_2() {
  velocity = input_velocity_1.Get();
  if (lock_checker < time.Get() - input_time_2.Get()) {
    curvature = input_curvature_3.Get();
  } else if (lock_checker < time.Get() - input_time_1.Get()) {
    curvature = input_curvature_2.Get();
  } else {
    curvature = input_curvature_1.Get();
  }
  if (lock_checker < unlock_value) {
    lock_checker++;
  } else {
    lock = false;
  }
  this->out_velocity.Publish(velocity);
  this->out_curvature.Publish(curvature);
  //this->out_velocity.Publish(1);
  //this->out_curvature.Publish(1.3);
}*/
//----------------------------------------------------------------------
// mEasyDrive drive_intersection
// Publishes the velocity and curvature necessary to drive
// through an intersection
//----------------------------------------------------------------------
void mEasyDrive::drive_intersection()
{
  //TODO
}
//----------------------------------------------------------------------
// mEasyDrive correction
// Publishes the velocity and curvature necessary to drive
// back onto the street when an error happen and the line got crossed
//----------------------------------------------------------------------
void mEasyDrive::expAlgrithm()
{
  /*
    int offset = 70;
    double x = line_error_test.Get() + offset;
    std::cout << x << std::endl;
    double a = 0.059;
    double function = 0;
    if (x >= 10)
    {
      function = -3 * (1 - exp(a * -x));
    }
    else if (x <= -10)
    {
      function = 3 * (1 - exp(a * x));
    }
    curvature = function;
  */
}

void mEasyDrive::expAlgrithm(int distance, double pixelValue)
{

  //offset = 70;
  double x = pixelValue + distance;
  //std::cout<<x<<std::endl;
  double a = 0.059;
  double function = 0;
  if (x >= 0)
  {
    function = -3 * (1 - exp(a * -x));
  }

  else
  {
    function = 3 * (1 - exp(a * x));
  }

  curvature = function;

}

void mEasyDrive::ruleBaseAlgrithm()
{
  /*if (x > right_boundary_1)
     {
     curvature = 1.5;
     }
     else if (x > right_boundary_2)
     {
     curvature = 1.3;
     }
     else if (x > right_boundary_3)
     {
     curvature = 1.0;
     }
     else if (x > right_boundary_4)
     {
     curvature = 0.7;
     }
     else if (x > right_boundary_5)
     {
     curvature = 0.4;
     }
     else if (x < -right_boundary_1)
     {
     curvature = -1.5;
     }
     else if (x < -right_boundary_2)
     {
     curvature = -1.3;
     }
     else if (x < -right_boundary_3)
     {
     curvature = -1.0;
     }
     else if (x < -right_boundary_4)
     {
     curvature = -0.7;
     }
     else if (x < -right_boundary_5)
     {
     curvature = -0.4;
     }
     else
     {
     curvature = 0;
     }*/
}

void mEasyDrive::linearAlgrithm()
{

}

void mEasyDrive::powerAlgrithm()
{
  //double x = std::get<2>(line_error.Get());
  //double function = c * (x*x*x) with x = (line_error + offset) /
  //for the middle line choose c = -0,1 and d = 19 and offset = 60
  //for the left line choose c = 0.1 and d = 19 and offset = 270 (not finished yet!!!!!)
  //for the right line choose c = ? and d = ? and offset = ?
  //shouldn it all be the same except for the offset? The change should have the same size right?
  /*int offset = time.Get();
  double d = 1;
  if (input_curvature_2.Get() != 0) {
    d = input_curvature_2.Get();
  }
  double x = (line_error_test.Get() + offset) / d;
  double c = input_curvature_1.Get();
  double function = c * (x * x * x);
  std::cout << "result: " << function << std::endl;
  curvature = function;*/

}
//----------------------------------------------------------------------
// mEasyDrive stop
// Publishes the velocity and curvature necessary to stop
//----------------------------------------------------------------------
/*void mEasyDrive::stop()
{
  velocity = 0;
  curvature = 0;
  if (lock_checker < unlock_value)
  {
    lock_checker++;
  }
  else
  {
    lock = false;
  }
  //this->out_velocity.Publish(velocity);
  this->out_curvature.Publish(curvature);
}
*/

void LineDetMachine::setLineValue(double m, double r, double l)
{

  midValue = m;
  rightValue = r;
  leftValue = l;
}

void LineDetMachine::chooseLine()
{
  switch (state)
  {
  case MID_LINE:
    if (midValue > midValue_min && midValue < midValue_max)
    {
      state = MID_LINE;
      //x = midValue;
      noLineDetection = false;
      //offset = 67;

      pixel = midValue;
      distance = rLane_mid_distance;

      std::cout << "Middle Line Choose : )))" << std::endl;
    }
    else
    {
      state = RIGHT_LINE;
    }
    break;

  case RIGHT_LINE:
    if (rightValue > rightValue_min && rightValue < rightValue_max)
    {
      state = RIGHT_LINE;
      noLineDetection = false;
      pixel = rightValue;
      distance = rLane_right_distance;
      std::cout << "Right Line Choose" << std::endl;
    }
    else if (midValue > midValue_min && midValue < midValue_max)
    {
      state = MID_LINE;
    }
    else
    {
      state = LEFT_LINE;
    }
    break;

  case LEFT_LINE:
    if (leftValue > leftValue_min && leftValue < leftValue_max)
    {
      state = LEFT_LINE;
      noLineDetection = false;
      pixel = leftValue;
      distance = rLane_left_distance;
      std::cout << "Left Line Choose" << std::endl;
    }
    else if (midValue > midValue_min && midValue < midValue_max)
    {
      state = MID_LINE;
    }
    else
    {
      state = STOP;
    }
    break;

  case STOP:
    distance = 0;
    pixel = 0;
    noLineDetection = true;
    std::cout << noLineDetection << std::endl;
    std::cout << "stop,,," << std::endl;
    if (midValue > midValue_min && midValue < midValue_max)
    {
      state = MID_LINE;
    }
    else if (rightValue > rightValue_min && rightValue < rightValue_max)
    {
      state = RIGHT_LINE;
    }
    else if (leftValue > leftValue_min && leftValue < leftValue_max)
    {
      state = LEFT_LINE;
    }

    else
    {
      state = STOP;
    }
    break;


  default:
    std::cout << "LineDetMachine screw up" << std::endl;
    break;
  }


}

double LineDetMachine::publishDistance()
{
  return distance;
}

double LineDetMachine::publishPixel()
{
  return pixel;
}

bool LineDetMachine::publishNoDetect()
{
  return noLineDetection;
}

std::tuple<int, double, bool> LineDetMachine::operation(double m, double r, double l)
{
  setLineValue(m, r, l);
  chooseLine();

  return std::make_tuple(publishDistance(), publishPixel(), publishNoDetect());
}


void IntersectDetMachine::chooseLine()
{
  switch (intersectState)
  {
  case MID_LINE:

    std::cout << "Intersection turn left" << std::endl;
    mc++;
    pixel = midValue;
    distance = rLane_mid_distance;

    if (mc >= interCounter)
    {
      intersectState = RIGHT_LINE;
      mc = 0;
      interProcessOn = false;

    }

    else if (midValue < midValue_min || midValue > midValue_max)
    {
      intersectState = STOP;
      stateMemory = MID_LINE;
      std::cout << "Intersection turn left NO DETECTION" << std::endl;
    }


    else
    {
      intersectState = MID_LINE;
    }

    break;

  case RIGHT_LINE:

    std::cout << "Intersection turn right" << std::endl;
    rc++;
    pixel = rightValue;
    distance = rLane_right_distance;

    if (rc >= interCounter)
    {
      intersectState = MID_LINE;
      rc = 0;
      interProcessOn = false;
    }

    else if (rightValue < rightValue_min || rightValue > rightValue_max)
    {
      intersectState = STOP;
      stateMemory = RIGHT_LINE;
      std::cout << "Intersection turn right NO DETECTION" << std::endl;
    }

    else
    {
      intersectState = RIGHT_LINE;
    }

    /*else
    {
      intersectState = RIGHT_LINE;
      std::cout << "Intersection turn right" << std::endl;
      rc++;
      pixel = rightValue;
      distance = rLane_right_distance;
    }*/
    break;

  case STOP:

    if (midValue >= midValue_min && midValue <= midValue_max && stateMemory == MID_LINE)
    {
      intersectState = MID_LINE;
      noLineDetection = false;
    }
    else if (rightValue >= rightValue_min && rightValue <= rightValue_max && stateMemory == RIGHT_LINE)
    {
      intersectState = RIGHT_LINE;
      noLineDetection = false;
    }
    else
    {
      distance = 0;
      pixel = 0;
      noLineDetection = true;
      std::cout << "Intersection stop" << stateMemory << std::endl;
    }
    break;

  default:
    std::cout << "Intersection state machine screw up" << std::endl;
    break;

  }

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
