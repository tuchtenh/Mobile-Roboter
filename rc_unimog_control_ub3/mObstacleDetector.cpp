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
/*!\file    projects/rc_unimog_control_ub3/mObstacleDetector.cpp
 *
 * \author  Lukas Tuchtenhagen, Marcel Suiker
 *
 * \date    2020-11-18
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_ub3/mObstacleDetector.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <iostream>
#include <cmath>
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
using namespace rrlib::si_units;
//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace rc_unimog_control_ub3
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
  tModule(parent, name, false)// change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)

{}

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

}

//----------------------------------------------------------------------
// mObstacleDetector OnParameterChange
//----------------------------------------------------------------------
void mObstacleDetector::OnParameterChange()
{
}

//----------------------------------------------------------------------
// mObstacleDetector Update
//----------------------------------------------------------------------
void mObstacleDetector::Update()
{
  if (this->InputChanged())
  {

      rrlib::math::tVec3f pointprev;
      data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> cloud = input_point_cloud.GetPointer();
      const rrlib::math::tVec3f* data = reinterpret_cast<const rrlib::math::tVec3f*>(cloud->DataPtr());
      for (unsigned int i = 0; i < cloud->Dimension(); i++)
      {
      rrlib::math::tVec3f point = data[i];
      // mit point.X(), point.Y() und point.Z() kann man auf die
      // Koordinaten des Punktes zugreifen

      //FINROC_LOG_PRINT(ERROR, distance);
      if (point.Z() >= 0){
          double distance = sqrt(point.X()*point.X() + point.Y()*point.Y() + point.Z()*point.Z());
          double distanceprev = sqrt((pointprev.X()-point.X())*(pointprev.X()-point.X()) + (pointprev.Y() -point.Y())*(pointprev.Y() -point.Y()) + (pointprev.Z() -point.Z())*(pointprev.Z() -point.Z()));
          double angle = std::acos(point.X()/std::sqrt(point.X()*point.X()+point.Y()*point.Y()));
          if (distance <= 1 && 0.009 >=distanceprev){
              //std::cout << point << std::endl;
              std::cout << "Distance " << distance << " Angle " << angle << std::endl;
              this->distance_out.Publish(distance);
              //std::cout << distanceprev << std::endl;
              this->angle_out.Publish(angle);
              this->boolOutput.Publish(true);//object detected
          }
          else{
              this->boolOutput.Publish(false);// no object detected
          }


      }
      pointprev = point;

      //FINROC_LOG_PRINT(ERROR, point.X());
      //FINROC_LOG_PRINT(ERROR, point.Y());
      //FINROC_LOG_PRINT(ERROR, point.Z());

      }
  }
  
  //Do something each cycle independent from changing ports.
  

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


/*

data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> cloud = input_point_cloud.GetPointer();
const rrlib::math::tVec3f* data = reinterpret_cast<const rrlib::math::tVec3f*>(cloud->DataPtr());
for (unsigned int i = 0; i < cloud->Dimension(); i++)
{
rrlib::math::tVec3f point = data[i];
// mit point.X(), point.Y() und point.Z() kann man auf die
// Koordinaten des Punktes zugreifen
...
}




// Farbinformation in Punktwolke enthalten? Kaiserslautern, Uni Ost
if (cloud->ExtraDataSize() >= cloud->Dimension() * 3)
{
const uint8_t* input_colors =
static_cast<const uint8_t*>(cloud->ExtraDataPtr());
...
// i ist der Index vom Punkt (s.o.)
uint8_t r = input_colors[i * 3];
uint8_t g = input_colors[i * 3 + 1];
uint8_t b = input_colors[i * 3 + 2];
}



*/
