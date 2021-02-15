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

    easyProcessOn = true;
    lineData = lineDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());

    distance = std::get<0>(lineData);
    pixel = std::get<1>(lineData);
    noDetect = std::get<2>(lineData);

    expStrongCurv(distance, pixel);


    if (coneDetect.Get() == true && overTakeDetPtr->coneProcessOn == false)
    {
      reactionState = CONE;
      overTakeDetPtr->coneProcessOn = true;
      std::cout << "Cone_Reaction_On" << std::endl;
    }


    else if (switchToLeft.Get() == true || giveWayDetect.Get() == true)
    {
      reactionState = SWITCH_TO_LEFT_MANUAL;
      std::cout << "Switch_To_Left" << std::endl;
    }

    else if (yellowSignDetect.Get() == true && straightOrLeft_int_temp != 1 && straightOrLeft_int_temp != 2)
    {
      reactionState = TURN_LEFT_GO_STRAIGHT_AUTO;
      intersectDetPtr->interProcessOn = true;
      std::cout << "Intersection_Auto" << std::endl;
    }

    else if (yellowSignDetect.Get() == true && straightOrLeft_int_temp == 1)
    {
      reactionState = TURN_LEFT_GO_STRAIGHT_MANUAL_LEFT;
      intersectDetPtr->interProcessOn = true;
      std::cout << "Intersection_Manual_Left" << std::endl;
    }

    else if (yellowSignDetect.Get() == true && straightOrLeft_int_temp == 2)
    {
      reactionState = TURN_LEFT_GO_STRAIGHT_MANUAL_STRAIGHT;
      intersectDetPtr->interProcessOn = true;
      std::cout << "Intersection_Manual_Straight" << std::endl;
    }

    else if (singleLaneDetect.Get())
    {
      reactionState = SINGLE;
      std::cout << "Single Lane Start" << std::endl;
    }

    else
    {
      reactionState = EASY;
    }

    break;

  case CONE:

    lineData = overTakeDetPtr->coneReaction(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());

    distance = std::get<0>(lineData);
    pixel = std::get<1>(lineData);
    noDetect = std::get<2>(lineData);

    expAlgrithm(distance, pixel);

    slowMotion_bool_temp = true;
    leftLight_temp = blinkFunction();

    if (overTakeDetPtr->coneProcessOn == false)
    {
      reactionState = EASY;
      std::cout << "Cone_Reaction_Off" << std::endl;
    }
    break;


  case SWITCH_TO_LEFT_MANUAL:

    if (coneDetect.Get())
    {
      overTakeDetPtr->coneDetectAtLeftLane = true;
    }

    if (switchToRight.Get())
    {
      overTakeDetPtr->switchToRight_temp = true;
    }
    if (giveWayDetect.Get())
    {
      overTakeDetPtr->giveWayDetect_temp = true;
    }


    changeSmoothCounter++;
    overTakeDetPtr->switchLeftProcessOn = true;

    if (changeSmoothCounter < 200)
    {
      slowMotion_bool_temp = true;
      lineData = overTakeDetPtr->operationToLeftLane(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());

      distance = std::get<0>(lineData);
      pixel = std::get<1>(lineData);
      noDetect = std::get<2>(lineData);

      expAlgrithm(distance, pixel);

      leftLight_temp = blinkFunction();

    }

    else
    {
      lineData = overTakeDetPtr->operationToLeftLaneExtreme(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
      changeSmoothCounter = 1000;

      distance = std::get<0>(lineData);
      pixel = std::get<1>(lineData);
      noDetect = std::get<2>(lineData);

      expStrongCurv(distance, pixel);
    }


    if (overTakeDetPtr->switchToRight_temp == true || overTakeDetPtr->giveWayDetect_temp == true)
    {
      changeSmoothCounter = 0;
      std::cout << "Switch_To_Right" << std::endl;

      if (overTakeDetPtr->smoothToRightTimer < 100) //smooth to right
      {
        overTakeDetPtr->switchLeftProcessOn = false;
        easyProcessOn = true;
        lineData = overTakeDetPtr->operationToRightLane(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
        distance = std::get<0>(lineData);
        pixel = std::get<1>(lineData);
        noDetect = std::get<2>(lineData);
        expStrongCurv(distance, pixel);
        overTakeDetPtr->smoothToRightTimer++;

        leftLight_temp = false;
        rightLight_temp = blinkFunction();
      }
      else
      {
        overTakeDetPtr->switchLeftProcessOn = false;
        reactionState = EASY;
        overTakeDetPtr->smoothToRightTimer = 0;
        overTakeDetPtr->switchToRight_temp = false;
        overTakeDetPtr->giveWayDetect_temp = false;

      }
    }// switchToRight.Get() || giveWayDetect.Get()

    //deal with cone at left lane
    else if (overTakeDetPtr->coneDetectAtLeftLane == true)
    {
      overTakeDetPtr->switchLeftProcessOn = false;
      overTakeDetPtr->coneProcessOn = true;
      changeSmoothCounter = 0;
      std::cout << "Detect Cone... Switch_To_Right" << std::endl;

      if (overTakeDetPtr->coneAtLeftLaneTimer < 200)
      {
        lineData = overTakeDetPtr->operationToRightLane(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
        distance = std::get<0>(lineData);
        pixel = std::get<1>(lineData);
        noDetect = std::get<2>(lineData);
        expStrongCurv(distance, pixel);
        overTakeDetPtr->coneAtLeftLaneTimer++;

        leftLight_temp = false;
        rightLight_temp = blinkFunction();
      }
      else
      {
        overTakeDetPtr->switchLeftProcessOn = false;
        overTakeDetPtr->coneProcessOn = false;
        overTakeDetPtr->coneDetectAtLeftLane = false;
        overTakeDetPtr->coneAtLeftLaneTimer = 0;
        reactionState = EASY;
      }
    }
    //deal with cone at left lane

    /*
     else if (yellowSignDetect.Get() == true)
     {





     }







    */



    break;

  case TURN_LEFT_GO_STRAIGHT_AUTO:

    slowMotion_bool_temp = true;

    lineData = intersectDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());

    distance = std::get<0>(lineData);
    pixel = std::get<1>(lineData);
    noDetect = std::get<2>(lineData);

    expStrongCurv(distance, pixel);
    leftLight_temp = blinkFunction();

    if (intersectDetPtr->interProcessOn == false)
    {
      reactionState = EASY;
      std::cout << "Intersection_Auto_Off" << std::endl;

      if (intersectDetPtr->goSingleLane == true)
      {
        reactionState = SINGLE;
      }
      else
      {
        reactionState = EASY;
      }

    }

    break;

  case TURN_LEFT_GO_STRAIGHT_MANUAL_LEFT:

    slowMotion_bool_temp = true;

    intersectDetPtr->setIntersectLeft();
    lineData = intersectDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());

    distance = std::get<0>(lineData);
    pixel = std::get<1>(lineData);
    noDetect = std::get<2>(lineData);

    expStrongCurv(distance, pixel);
    leftLight_temp = blinkFunction();

    if (intersectDetPtr->interProcessOn == false)
    {
      reactionState = EASY;
      std::cout << "Intersection_Manual_Left_Off" << std::endl;
    }

    break;

  case TURN_LEFT_GO_STRAIGHT_MANUAL_STRAIGHT:

    slowMotion_bool_temp = true;

    intersectDetPtr->setIntersectStraight();
    lineData = intersectDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());

    distance = std::get<0>(lineData);
    pixel = std::get<1>(lineData);
    noDetect = std::get<2>(lineData);

    expStrongCurv(distance, pixel);

    if (intersectDetPtr->interProcessOn == false)
    {
      //reactionState = EASY;
      reactionState = SINGLE;
      std::cout << "Intersection_Manual_Straight_Off" << std::endl;
    }

    break;

  case SINGLE:

    singleDetPtr->laneWidth = input_curvature_right.Get() - input_curvature_left.Get();

    singleMotion_bool_temp = true;
    singleDetPtr->singleProcessOn = true;

    lineData = singleDetPtr->operation(input_curvature_middle.Get(), input_curvature_right.Get(), input_curvature_left.Get());
    distance = std::get<0>(lineData);
    pixel = std::get<1>(lineData);
    noDetect = std::get<2>(lineData);

    expStrongCurv(distance, pixel);

    if (easy.Get())
    {
      intersectDetPtr->goSingleLane = false;
      reactionState = EASY;
      singleDetPtr->singleProcessOn = false;
      std::cout << "Back To Easy" << std::endl;
    }


    else if ((singleLaneDetect.Get() == false) && (singleDetPtr->laneWidth >= singleDetPtr->doubleLaneWidth_min) && (singleDetPtr->laneWidth <= singleDetPtr->doubleLaneWidth_max))
    {

      intersectDetPtr->goSingleLane = false;
      reactionState = EASY;
      singleDetPtr->singleProcessOn = false;
      std::cout << "EasyDrive" << std::endl;
    }

  default:
    break;

  }// switch  reactionState




  if (noDetect == true) //three lines detection fail
  {
    curvature = 0;
    std::cout << "Line distance is infinite =) =)" << std::endl;

    stopLight_temp = true;
  }

  else
  {
    //expAlgrithm(distance, pixel);
    //velocity = 0.8;
  }


  //if( noDetect == true && (overTakeDetPtr->coneProcessOn == true || overTakeDetPtr->switchLeftProcessOn == true)) // switch color
  if (noDetect == true && (reactionState == CONE || reactionState == SWITCH_TO_LEFT_MANUAL)) // switch color
  {
    colorSwitch = true;
  }
  if (reactionState == EASY)
  {
    colorSwitch = false;
  }

  //to velocity controller
  this->out_slowMotion.Publish(slowMotion_bool_temp);
  this->out_singleMotion.Publish(singleMotion_bool_temp);
  this->out_curvature.Publish(curvature);
  this->out_noLineDetection.Publish(noDetect);

  //to ZED
  this->out_colorSwitch.Publish(colorSwitch);

  //publish light
  lights_out_temp[1] = stopLight_temp;
  lights_out_temp[2] = leftLight_temp;
  lights_out_temp[3] = rightLight_temp;

  lights_out.Publish(lights_out_temp);


  gui_Easy_temp = easyProcessOn;
  gui_Intersect_temp = intersectDetPtr->interProcessOn;
  gui_Cone_temp = overTakeDetPtr->coneProcessOn;
  gui_LeftLane_temp = overTakeDetPtr->switchLeftProcessOn;
  gui_Single_temp = singleDetPtr->singleProcessOn;

  this->gui_Easy.Publish(gui_Easy_temp);
  this->gui_Intersect.Publish(gui_Intersect_temp);
  this->gui_Cone.Publish(gui_Cone_temp);
  this->gui_LeftLane.Publish(gui_LeftLane_temp);
  this->gui_Single.Publish(gui_Single_temp);



}

void mEasyDrive::intitialization()
{
  slowMotion_bool_temp = false;
  singleMotion_bool_temp = false;
  easyProcessOn = false;
  stopLight_temp = false;
}

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

void mEasyDrive::expSwitchLane(int distance, double pixelValue)
{


  double x = pixelValue + distance;
  double a_p = 0.005;
  double a_n = 0.01;
  double function = 0;
  if (x >= 0)
  {
    function = -3 * (1 - exp(a_n * -x));
  }

  else
  {
    function = 3 * (1 - exp(a_p * x));
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

  if (function >= 1.3)
  {
    function = 3;
  }
  else if (function <= -1.3)
  {
    function = -3;
  }
  curvature = function;

}

void mEasyDrive::ruleBaseAlgrithm()
{
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

bool mEasyDrive::blinkFunction()
{
  blinkTimer_temp++;
  if (blinkTimer_temp < (blinkTimer / 2))
  {
    return true;
  }

  else if (blinkTimer_temp >= (blinkTimer / 2) && blinkTimer_temp < blinkTimer)
  {
    return false;
  }
  else
  {
    blinkTimer_temp = 0;
    return false;
  }

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
      noLineDetection = false;
      pixel = midValue;
      distance = rLane_mid_distance;
      std::cout << "Mid Line Choose..." << std::endl;
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

void IntersectDetMachine::setIntersectLeft()
{
  intersectState = MID_LINE;

}

void IntersectDetMachine::setIntersectStraight()
{
  intersectState = RIGHT_LINE;

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

      goSingleLane = true;  ///////////////////////////////////////////////////////////
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

}


void OverTakeDetMachine::chooseLineToLeftExtreme()
{
  switch (overTakeState)
  {
  case LEFT_LINE:

    if (leftValue > lLeftValue_min && leftValue < lLeftValue_max)
    {
      overTakeState = LEFT_LINE;
      noLineDetection = false;
      pixel = leftValue;
      distance = lLane_left_extreme_distance;
      std::cout << "Left Line Choose for OverTakeToLeft Extreme" << std::endl;
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
      distance = lLane_right_extreme_distance;
      std::cout << "Right Line Choose for OverTakeToLeft Extreme" << std::endl;
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
    std::cout << "stop, for OverTakeToLeft Extreme" << std::endl;

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
    std::cout << "OverTakeDetMachineToLeftExtreme screw up" << std::endl;
    break;
  }


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
      std::cout << "Right Line Choose for OverTakeToRight" << std::endl;
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


std::tuple<int, double, bool> OverTakeDetMachine::operationToLeftLaneExtreme(double m, double r, double l)
{
  setLineValue(m, r, l);
  chooseLineToLeftExtreme();

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

  if (coneReactionTimer <= 220)
  {
    std::tuple<int, double, bool> temp = operationToLeftLane(m, r, l);
    if ((std::get<2>(temp)) == false)
    {
      coneReactionTimer++;
    }

    return temp;
  }

  else if (coneReactionTimer >= 220 && coneReactionTimer < 350)
  {
    std::tuple<int, double, bool> temp = operationToLeftLaneExtreme(m, r, l);
    if ((std::get<2>(temp)) == false)
    {
      coneReactionTimer++;
    }

    return temp;
  }

  else if (coneReactionTimer >= 350 && coneReactionTimer < 500)
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
    coneProcessOn = false;
    coneReactionTimer = 0;
    return operationToRightLane(m, r, l);
  }



}


void SingleDetMachine::chooseLine()
{
  switch (SingleState)
  {
  case RIGHT_LINE:
    if (rightValue > rightValue_min && rightValue < rightValue_max)
    {
      SingleState = RIGHT_LINE;
      noLineDetection = false;
      pixel = rightValue;
      distance = single_right_distance;
      std::cout << "Right Line Choose for SingleLane" << std::endl;
    }

    else
    {
      SingleState = LEFT_LINE;
    }
    break;

  case LEFT_LINE:
    if (leftValue > leftValue_min && leftValue < leftValue_max)
    {
      SingleState = LEFT_LINE;
      noLineDetection = false;
      pixel = leftValue;
      distance = single_left_distance;
      std::cout << "Left Line Choose for SingleLane" << std::endl;
    }
    else if (rightValue > rightValue_min && rightValue < rightValue_max)
    {
      SingleState = RIGHT_LINE;
    }
    else
    {
      SingleState = STOP;
    }
    break;

  case STOP:
    distance = 0;
    pixel = 0;
    noLineDetection = true;

    std::cout << "Single Lane stop" << std::endl;

    if (rightValue > rightValue_min && rightValue < rightValue_max)
    {
      SingleState = RIGHT_LINE;
    }
    else if (leftValue > leftValue_min && leftValue < leftValue_max)
    {
      SingleState = LEFT_LINE;
    }

    else
    {
      SingleState = STOP;
    }
    break;


  default:
    std::cout << "Sinle Lane Machine screw up screw up" << std::endl;
    break;
  }

}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
