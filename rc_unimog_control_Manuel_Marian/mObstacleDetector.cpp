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
/*!\file    projects/rc_unimog_control_zed2/mObstacleDetector.cpp
 *
 * \author  Manuel Vogel
 *
 * \date    2020-11-19
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_zed2/mObstacleDetector.h"

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

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace rc_unimog_control_zed2
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mObstacleDetector> cCREATE_ACTION_FOR_M_OBSTACLEDETECTOR("ObstacleDetector");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mObstacleDetector constructor
//----------------------------------------------------------------------
mObstacleDetector::mObstacleDetector(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false), // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
  object_in_way(false),
  distance_to_object(777.0),
  angle_to_object(0.0)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
{
  //parent->hardware
}

//----------------------------------------------------------------------
// mObstacleDetector destructor
//----------------------------------------------------------------------
mObstacleDetector::~mObstacleDetector()
{}

//----------------------------------------------------------------------
// mObstacleDetector OnStaticParameterChange
//----------------------------------------------------------------------
void mObstacleDetector::OnStaticParameterChange()
{
  /*if (this->static_parameter_1.HasChanged())
  {
    As this static parameter has changed, do something with its value!
  }*/
}

//----------------------------------------------------------------------
// mObstacleDetector OnParameterChange
//----------------------------------------------------------------------
void mObstacleDetector::OnParameterChange()
{
  //If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
}

//----------------------------------------------------------------------
// mObstacleDetector Update
//----------------------------------------------------------------------
void mObstacleDetector::Update()
{
  
  if (this->InputChanged())
  {
    object = false;
    distanceToObject = 10.0;
    angleToObject = 0.0;
    counter = 0;
    data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData>
      cloud = input_point_cloud.GetPointer();
    const rrlib::math::tVec3f* data =
      reinterpret_cast<const rrlib::math::tVec3f*>(cloud->DataPtr());

    for (unsigned int i = 0; i < cloud->Dimension(); i++)
    {
      rrlib::math::tVec3f point = data[i];
      if(point.X() < 3.0 && point.Z() > 0.01){  //if Point is 1cm above ground and less than 2 m away
        counter++;
	if( sqrt(point.X()*point.X() + point.Y() * point.Y() + point.Z() * point.Z()) < distanceToObject){
	
		distanceToObject = std::min(distanceToObject,  sqrt(point.X()*point.X() + point.Y() * point.Y() + point.Z() * point.Z()));
		angleToObject = atan2(point.Y(),point.X());
		}
        }
    }

    if(counter > 100 && distanceToObject < 0.7){
      object = true;
    }
  }
  this->object_in_way.Publish(object);
  this->distance_to_object.Publish(distanceToObject);
  this->angle_to_object.Publish(angleToObject);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
