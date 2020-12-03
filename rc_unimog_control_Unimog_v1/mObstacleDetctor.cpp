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
/*!\file    projects/rc_unimog_control_Unimog_v1/mObstacleDetctor.cpp
 *
 * \author  Aaron Hackenberg
 *
 * \date    2020-11-19
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_Unimog_v1/mObstacleDetctor.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <iostream>
#include <vector>
#include <string>
#include <tuple>
#include <math.h>
#include <stdio.h>
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>


#define PI 3.14159265

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace rc_unimog_control_Unimog_v1
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mObstacleDetctor> cCREATE_ACTION_FOR_M_OBSTACLEDETCTOR("ObstacleDetctor");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mObstacleDetctor constructor
//----------------------------------------------------------------------
mObstacleDetctor::mObstacleDetctor(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false),
  current_length(0),
  current_angle(0)
  //enable(false)
  //close_points = new vector<tuple<int, int>>()// change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
{}

//----------------------------------------------------------------------
// mObstacleDetctor destructor
//----------------------------------------------------------------------
mObstacleDetctor::~mObstacleDetctor()
{}

//----------------------------------------------------------------------
// mObstacleDetctor OnStaticParameterChange
//----------------------------------------------------------------------
void mObstacleDetctor::OnStaticParameterChange()
{
  /*if (this->static_parameter_1.HasChanged())
  {
    As this static parameter has changed, do something with its value!
  }
  */
}

//----------------------------------------------------------------------
// mObstacleDetctor OnParameterChange
//----------------------------------------------------------------------
void mObstacleDetctor::OnParameterChange()
{
  //If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
}

//----------------------------------------------------------------------
// mObstacleDetctor Update
//----------------------------------------------------------------------
void mObstacleDetctor::Update()
{
  /*if (this->InputChanged())
  {
    At least one of your input ports has changed. Do something useful with its data.
    However, using the .HasChanged() method on each port you can check in more detail.
  }
*/
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData>cloud = input_point_cloud.GetPointer();
  const rrlib::math::tVec3f* data =reinterpret_cast<const rrlib::math::tVec3f*>(cloud->DataPtr());
  for (unsigned int i = 0; i < cloud->Dimension(); i++)
  {
	  rrlib::math::tVec3f point = data[i];// mit point.X(), point.Y() und point.Z() kann man auf die// Koordinaten des Punktes zugreifen...
	  float temp = sqrtf(point.X() * point.X() + point.Y() * point.Y() + point.Z()*point.Z());
	  if (temp < 1.5 && point.Z() >= 0.0 && point.Y() >= -0.5 && point.Y() <= 0.3)
	  {
		  close_points.push_back(std::tuple<float, float>(point.Y(), temp));
		  /*std::cout << "X: " << point.X() << std::endl;
		  std::cout << "Y: " << point.Y() << std::endl;
		  std::cout << "Z: " << point.Z() << "=" << sqrtf(point.X() * point.X() + point.Y() * point.Y() + point.Z()*point.Z()) << std::endl;*/
	  }
  }
  if (!close_points.empty()) {
	  this->out_object.Publish(true);
  } else {
	  this->out_object.Publish(false);
  }
  Closest_object();
  close_points.clear();
  this->distance.Publish(current_length);
  this->angle.Publish(current_angle);
}

void mObstacleDetctor::Closest_object() {
	float* min = &std::get<1>(close_points.back());
	float* y = &std::get<0>(close_points.back());
	enable = false;
	for(std::tuple<float, float> t : close_points) {
		enable = true;
		if (std::get<1>(t) < *min) {
			min = &std::get<1>(t);
			y = &std::get<0>(t);
		}
	}
	//std::cout << *min;
	if (enable) {
		current_length = *min;
		//rrlib::math::tAngleRad t_angle = {asin(*y / *min)};
		double t_angle = asin(*y / *min);
		current_angle = t_angle;
		std::cout << "depth: " << current_length << std::endl;
		std::cout << "Y: " << *y << std::endl;
		std::cout << "angle: " << t_angle << std::endl;
	}
	//current_length = *min;
	//std::cout << "test";
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
