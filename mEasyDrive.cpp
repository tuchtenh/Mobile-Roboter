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
  tModule(parent, name, false), velocity(0), curvature(0), lock(false)/*,
 right_boundary_1(90),
 right_boundary_2(70),
 right_boundary_3(50),
 right_boundary_4(30),
 right_boundary_5(10)*/
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
  //std::cout <<  "AAAAA"<<std::endl;
  // choose which line to follow
  lineDet.setLineValue(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
  lineDet.chooseLine();
  int offset = lineDet.publishOffset();
  double x = lineDet.publishPixel();

  //follow the line
  //if (enable.Get())
  //{
  /*if (std::isinf(line_error_test.Get())) {
    velocity = 0;
    curvature = 0;
    std::cout<<"Line distance is infinite!"<<std::endl;
  } else {
    expAlgrithm();
    velocity = 0.4;
  }*/

  if (lineDet.noLineDetection == true) //three lines detection fail
  {
    curvature = 0;
    std::cout << "Line distance is infinite!" << std::endl;
  }
  else
  {
    expAlgrithm(offset, x);
    velocity = 0.8;
  }



  //this->out_velocity.Publish(velocity);
  this->out_curvature.Publish(curvature);
  this->out_noLineDetection.Publish(lineDet.noLineDetection);
  /*
   if (mode.Get() == 1) {
   if (!lock) {
   lock = true;
   lock_checker = 0;
   unlock_value = 245;
   }
   drive_straight();
   } else if (mode.Get() == 2) {
   if (!lock) {
   lock = true;
   lock_checker = 0;
   unlock_value = time.Get();
   }
   drive_curve_1();
   } else if(mode.Get() == 3) {
   if (!lock) {
   lock = true;
   lock_checker = 0;
   unlock_value = time.Get();
   }
   drive_curve_2();
   } else {
   if (!lock) {
   lock = true;
   lock_checker = 0;
   unlock_value = 1000;
   }
   stop();
   }
   */

  //check for the lock, if lock=true, Publish the same velocity and curvature as last time.
  /*if (lock && lock_checker < unlock_value) {
   if(mode.Get() == 2) {
   if (lock_checker < input_time_1.Get()) {
   curvature = input_curvature_1.Get();
   } else if (lock_checker < input_time_2.Get()){
   curvature = input_curvature_2.Get();
   } else {
   curvature = input_curvature_3.Get();
   }
   }
   lock_checker++;
   } else {
   lock = false;
   lock_checker = 0;
   //call the function fitting the current road type
   if (mode.Get() == 1) {
   if (!curve) {
   curve = true;
   drive_straight();
   } else {
   curve=false;
   stop();
   }
   } else if (mode.Get() == 2) {
   if (!curve) {
   curve = true;
   drive_curve();
   } else {
   curve=false;
   stop();
   }
   } else {
   stop();
   }
   }
   this->out_velocity.Publish(velocity);
   this->out_curvature.Publish(curvature);*/
  //}
  //TODO
  /*if (this->InputChanged())
   {
   At least one of your input ports has changed. Do something useful with its data.
   However, using the .HasChanged() method on each port you can check in more detail.
   }

   Do something each cycle independent from changing ports.

   this->out_signal_1.Publish(some meaningful value); can be used to publish data via your output ports.*/
}

//----------------------------------------------------------------------
// mEasyDrive drive_straight
// Publishes the velocity and curvature necessary to drive
// in a straight line within a lane
//----------------------------------------------------------------------
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

}
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

}

void mEasyDrive::expAlgrithm(int offset, double pixelValue)
{

  //offset = 70;
  double x = pixelValue + offset;
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
void mEasyDrive::stop()
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

      x = midValue; //(midValue+67)/2-67; for low speed
      offset = mid_offset;

      std::cout << "Middle Line Choose =)))" << std::endl;
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
      x = rightValue;
      offset = right_offset;
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
      x = leftValue;
      offset = left_offset;
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
    offset = 0;
    x = 0;
    noLineDetection = true;
    std::cout << noLineDetection << std::endl;
    std::cout << "stop" << std::endl;
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
    std::cout << "screw up" << std::endl;
    break;
  }


}

double LineDetMachine::publishOffset()
{
  return offset;
}

double LineDetMachine::publishPixel()
{
  return x;
}
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
