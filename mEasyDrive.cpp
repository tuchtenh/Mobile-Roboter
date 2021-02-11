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
  tModule(parent, name, false), curvature(0)
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

  std::tuple<int, double, bool> lineData(0, 0, false);

  int distance;
  double pixel;
  bool noDetect;

  intitialization();
  getDetectionAndManual();

  switch (reactionState)
  {
  case EASY:

    lineData = lineDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());

    distance = std::get<0>(lineData);
    pixel = std::get<1>(lineData);
    noDetect = std::get<2>(lineData);

    expStrongCurv(distance, pixel);


    if (coneDetect.Get() == true)
    {
      reactionState = CONE;
      overTakeDetPtr->takeoverProcessOn = true;
      std::cout << "Cone_Reaction_On" << std::endl;
    }

    if (switchToLeft.Get() == true)
    {
      reactionState = SWITCH_TO_LEFT_MANUAL;
      std::cout << "Switch_To_Left" << std::endl;
    }

    if (yellowSignDetect.Get() == true && straightOrLeft_int_temp != 1 && straightOrLeft_int_temp != 2)
    {
      reactionState = TURN_LEFT_GO_STRAIGHT_AUTO;
      intersectDetPtr->interProcessOn = true;
      std::cout << "Intersection_Auto" << std::endl;
    }

    break;

  case CONE:

    lineData = overTakeDetPtr->coneReaction(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());

    distance = std::get<0>(lineData);
    pixel = std::get<1>(lineData);
    noDetect = std::get<2>(lineData);

    expAlgrithm(distance, pixel);

    if (overTakeDetPtr->takeoverProcessOn == false)
    {
      reactionState = EASY;
      std::cout << "Cone_Reaction_Off" << std::endl;
    }
    break;


  case SWITCH_TO_LEFT_MANUAL:

    lineData = overTakeDetPtr->operationToLeftLane(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());

    distance = std::get<0>(lineData);
    pixel = std::get<1>(lineData);
    noDetect = std::get<2>(lineData);

    expAlgrithm(distance, pixel);

    if (switchToRight.Get() || coneDetect.Get())
    {
      reactionState = EASY;
      std::cout << "Switch_To_Right" << std::endl;
    }


    break;

  case TURN_LEFT_GO_STRAIGHT_AUTO:

    lineData = intersectDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());

    distance = std::get<0>(lineData);
    pixel = std::get<1>(lineData);
    noDetect = std::get<2>(lineData);

    expStrongCurv(distance, pixel);

    if (intersectDetPtr->interProcessOn == false)
    {
      reactionState = EASY;
      std::cout << "Intersection_Off" << std::endl;

    }

    break;

  case TURN_LEFT_GO_STRAIGHT_MANUAL:


    break;



  default:
    break;

  }


  //lineData = lineDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
  //lineData = overTakeDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
  /*
   //ConeReaction
    if (test_bool.Get() == true)
    {
      std::cout << "ConeReactionOn" << std::endl;
      lineData = overTakeDetPtr->coneReaction(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());


       distance = std::get<0>(lineData);
       pixel = std::get<1>(lineData);
       noDetect = std::get<2>(lineData);

      expAlgrithm(distance, pixel);
    }
    else
    {
      lineData = lineDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());

       distance = std::get<0>(lineData);
       pixel = std::get<1>(lineData);
       noDetect = std::get<2>(lineData);

      expStrongCurv(distance, pixel);
    }
  */

  /*
  //change lanes
  if (test_bool.Get() == false)
  {
    //lineData = lineDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
    lineData = overTakeDetPtr->operationToRightLane(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
  }

  else
  {
    lineData = overTakeDetPtr->operationToLeftLane(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
  }
  */


  /*
  //Turn left or go straight
   *
    if (test_bool.Get() == true)
    {
      intersectDetPtr->interProcessOn = true;
    }


    if (intersectDetPtr->interProcessOn == true)
    {
      lineData = intersectDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
    }

    else
    {
      lineData = lineDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
    }

  */

  /*
    int distance = std::get<0>(lineData);
    double pixel = std::get<1>(lineData);
    bool noDetect = std::get<2>(lineData);
  */

  // EasyDrive

  /*
  lineData = lineDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());

  distance = std::get<0>(lineData);
  pixel = std::get<1>(lineData);
  noDetect = std::get<2>(lineData);

  expStrongCurv(distance, pixel);
  */

  if (noDetect == true) //three lines detection fail
  {
    curvature = 0;
    std::cout << "Line distance is infinite =) =)" << std::endl;
  }

  else
  {
    //expAlgrithm(distance, pixel);
    //velocity = 0.8;
  }




  this->out_curvature.Publish(curvature);
  this->out_noLineDetection.Publish(noDetect);


}

void mEasyDrive::intitialization()
{}

void mEasyDrive::getDetectionAndManual()
{
  straightOrLeft_int_temp = straightOrLeft.Get();


}



//----------------------------------------------------------------------
// mEasyDrive correction
// Publishes the velocity and curvature necessary to drive
// back onto the street when an error happen and the line got crossed
//----------------------------------------------------------------------


void mEasyDrive::expAlgrithm(int distance, double pixelValue)
{


  double x = pixelValue + distance;
  double a = 0.01;
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

void mEasyDrive::expStrongCurv(int distance, double pixelValue)
{


  double x = pixelValue + distance;
  double a = 0.01;
  double function = 0;
  if (x >= 0)
  {
    function = -3 * (1 - exp(a * -x));
  }

  else
  {
    function = 3 * (1 - exp(a * x));
  }

  if (function >= 1.5)
  {
    function = 3;
  }
  else if (function <= -1.5)
  {
    function = -3;
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

void mEasyDrive::powerAlgrithm(int distance, double pixelValue)
{
  //double x = pixelValue + distance;

  double function = 0;
  /*if (x >= 0)
  {
    function = -3 * (1 - exp(a * -x));
  }

  else
  {
    function = 3 * (1 - exp(a * x));
  }*/

  curvature = function;

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
  case RIGHT_LINE:
    if (rightValue > rightValue_min && rightValue < rightValue_max)
    {
      state = RIGHT_LINE;
      noLineDetection = false;
      pixel = rightValue;
      distance = rLane_right_distance;
      std::cout << "Right Line Choose" << std::endl;
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
    else if (rightValue > rightValue_min && rightValue < rightValue_max)
    {
      state = RIGHT_LINE;
    }
    else
    {
      state = MID_LINE;
    }
    break;

  case MID_LINE:
    if (midValue > midValue_min && midValue < midValue_max)
    {
      state = MID_LINE;
      noLineDetection = false;
      pixel = midValue;
      distance = rLane_mid_distance;
      std::cout << "Mid Line Choose" << std::endl;
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

  case STOP:
    distance = 0;
    pixel = 0;
    noLineDetection = true;

    std::cout << "EasyDrive stop" << std::endl;

    if (rightValue > rightValue_min && rightValue < rightValue_max)
    {
      state = RIGHT_LINE;
    }
    else if (leftValue > leftValue_min && leftValue < leftValue_max)
    {
      state = LEFT_LINE;
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


  default:
    std::cout << "LineDetMachine screw up" << std::endl;
    break;

  }

  /*
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

  }*/


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

    std::cout << "Intersection turn left MID_LINE" << std::endl;
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
      intersectState = LEFT_LINE;
      stateMemory = MID_LINE;
      std::cout << "Intersection turn left LEFT_LINE" << std::endl;
    }

    else
    {
      intersectState = MID_LINE;
    }
    break;

  case LEFT_LINE:
    std::cout << "Intersection turn left LEFT_LINE" << std::endl;
    mc++;
    pixel = leftValue;
    distance = rLane_left_distance;

    if (mc >= interCounter || (midValue >= midValue_min && midValue <= midValue_max))
    {
      intersectState = MID_LINE;
    }
    else if (leftValue < leftValue_min || leftValue > leftValue_max)
    {
      intersectState = STOP;
      stateMemory = MID_LINE;
    }
    else
    {
      intersectState = LEFT_LINE;
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


    break;

  case STOP:

    if (midValue >= midValue_min && midValue <= midValue_max && stateMemory == MID_LINE)
    {
      intersectState = MID_LINE;
      noLineDetection = false;
    }

    else if (leftValue >= leftValue_min && leftValue <= leftValue_max && stateMemory == MID_LINE)
    {
      intersectState = LEFT_LINE;
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
      std::cout << "Intersection stop  " << stateMemory << std::endl;
    }
    break;

  default:
    std::cout << "Intersection state machine screw up" << std::endl;
    break;
  }

  /*
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

  }*/

}

void OverTakeDetMachine::chooseLineToLeft()
{
  switch (overTakeState)
  {
  case LEFT_LINE:

    if (leftValue > lLeftValue_min && leftValue < lLeftValue_max)
    {
      overTakeState = LEFT_LINE;
      noLineDetection = false;
      pixel = leftValue;
      distance = lLane_left_distance;
      std::cout << "Left Line Choose for OverTakeToLeft" << std::endl;
    }
    else
    {
      overTakeState = RIGHT_LINE;
    }

    break;

  case RIGHT_LINE:

    if (rightValue > lRightValue_min && rightValue < lRightValue_max)
    {
      overTakeState = RIGHT_LINE;
      noLineDetection = false;
      pixel = rightValue;
      distance = lLane_right_distance;
      std::cout << "Right Line Choose for OverTakeToLeft" << std::endl;
    }
    else if (leftValue > lLeftValue_min && leftValue < lLeftValue_max)
    {
      overTakeState = LEFT_LINE;
    }
    else
    {
      overTakeState = STOP;
    }
    break;



  case STOP:

    distance = 0;
    pixel = 0;
    noLineDetection = true;
    std::cout << "stop, for OverTakeToLeft" << std::endl;

    if (leftValue > lLeftValue_min && leftValue < lLeftValue_max)
    {
      overTakeState = LEFT_LINE;
    }
    else if (rightValue > lRightValue_min && rightValue < lRightValue_max)
    {
      overTakeState = RIGHT_LINE;
    }

    else
    {
      overTakeState = STOP;
    }
    break;


  default:
    std::cout << "OverTakeDetMachineToLeft screw up" << std::endl;
    break;
  }
  /*
  switch (state)
  {
  case MID_LINE:
    if (midValue > lMidValue_min && midValue < lMidValue_max)
    {
      state = MID_LINE;
      //x = midValue;
      noLineDetection = false;
      //offset = 67;

      pixel = midValue;
      distance = lLane_mid_distance;

      std::cout << "Middle Line Choose for OverTake: )))" << std::endl;
    }
    else
    {
      state = RIGHT_LINE;
    }
    break;

  case RIGHT_LINE:
    if (rightValue > lRightValue_min && rightValue < lRightValue_max)
    {
      state = RIGHT_LINE;
      noLineDetection = false;
      pixel = rightValue;
      distance = lLane_right_distance;
      std::cout << "Right Line Choose for OverTake" << std::endl;
    }
    else if (midValue > lMidValue_min && midValue < lMidValue_max)
    {
      state = MID_LINE;
    }
    else
    {
      state = LEFT_LINE;
    }
    break;

  case LEFT_LINE:
    if (leftValue > lLeftValue_min && leftValue < lLeftValue_max)
    {
      state = LEFT_LINE;
      noLineDetection = false;
      pixel = leftValue;
      distance = lLane_left_distance;
      std::cout << "Left Line Choose for OverTake" << std::endl;
    }
    else if (midValue > lMidValue_min && midValue < lMidValue_max)
    {
      state = RIGHT_LINE;
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
    std::cout << "stop,,, for Intersect" << std::endl;
    if (midValue > lMidValue_min && midValue < lMidValue_max)
    {
      state = MID_LINE;
    }
    else if (rightValue > lRightValue_min && rightValue < lRightValue_max)
    {
      state = RIGHT_LINE;
    }
    else if (leftValue > lLeftValue_min && leftValue < lLeftValue_max)
    {
      state = LEFT_LINE;
    }

    else
    {
      state = STOP;
    }
    break;


  default:
    std::cout << "OverTakeDetMachine screw up" << std::endl;
    break;
  }

  */

}


void OverTakeDetMachine::chooseLineToRight()
{
  switch (overTakeState)
  {
  case RIGHT_LINE:
    if (rightValue > rightValue_min && rightValue < rightValue_max)
    {
      overTakeState = RIGHT_LINE;
      noLineDetection = false;
      pixel = rightValue;
      distance = rLane_right_distance;
      std::cout << "Left Line Choose for OverTakeToRight" << std::endl;
    }

    else
    {
      overTakeState = LEFT_LINE;
    }
    break;

  case LEFT_LINE:
    if (leftValue > leftValue_min && leftValue < leftValue_max)
    {
      overTakeState = LEFT_LINE;
      noLineDetection = false;
      pixel = leftValue;
      distance = rLane_left_distance;
      std::cout << "Left Line Choose for OverTakeToRight" << std::endl;
    }
    else if (rightValue > rightValue_min && rightValue < rightValue_max)
    {
      overTakeState = RIGHT_LINE;
    }
    else
    {
      overTakeState = STOP;
    }
    break;

  case STOP:
    distance = 0;
    pixel = 0;
    noLineDetection = true;

    std::cout << "OverTakeToRight stop" << std::endl;

    if (rightValue > rightValue_min && rightValue < rightValue_max)
    {
      overTakeState = RIGHT_LINE;
    }
    else if (leftValue > leftValue_min && leftValue < leftValue_max)
    {
      overTakeState = LEFT_LINE;
    }

    else
    {
      overTakeState = STOP;
    }
    break;


  default:
    std::cout << "OverTakeDetMachineToRight screw up screw up" << std::endl;
    break;
  }

}

std::tuple<int, double, bool> OverTakeDetMachine::operationToLeftLane(double m, double r, double l)
{
  setLineValue(m, r, l);
  chooseLineToLeft();

  return std::make_tuple(publishDistance(), publishPixel(), publishNoDetect());
}

std::tuple<int, double, bool> OverTakeDetMachine::operationToRightLane(double m, double r, double l)
{
  setLineValue(m, r, l);
  chooseLineToRight();

  return std::make_tuple(publishDistance(), publishPixel(), publishNoDetect());
}

std::tuple<int, double, bool> OverTakeDetMachine::coneReaction(double m, double r, double l)
{

  if (coneReactionTimer <= 300)
  {
    std::tuple<int, double, bool> temp = operationToRightLane(m, r, l);
    if ((std::get<2>(temp)) == false)
    {
      coneReactionTimer++;
    }

    return temp;
  }

  else if (coneReactionTimer > 300 && coneReactionTimer < 600)
  {
    std::tuple<int, double, bool> temp = operationToRightLane(m, r, l);
    if ((std::get<2>(temp)) == false)
    {
      coneReactionTimer++;
    }

    return temp;
  }

  else
  {
    takeoverProcessOn = false;
    coneReactionTimer = 0;
    return operationToRightLane(m, r, l);
  }



}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
