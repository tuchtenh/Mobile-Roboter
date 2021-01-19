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
/*!\file    projects/finroc_projects_robprak2020_2/mImageDetectorYolo.h
 *
 * \author  Manuel Vogel
 *
 * \date    2021-01-08
 *
 * \brief Contains mImageDetectorYolo
 *
 * \b mImageDetectorYolo
 *
 * this uses tinyYolo v4 to detect stop signs and stuff
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__finroc_projects_robprak2020_2__mImageDetectorYolo_h__
#define __projects__finroc_projects_robprak2020_2__mImageDetectorYolo_h__

#include "plugins/structure/tModule.h"


#include "rrlib/geometry/tBoundingBox.h"
//#include "rrlib/machine_learning_appliance/tMLDetection.h"

//#include "libraries/object_detection/tDarknetYOLO.h"
//#include "libraries/object_detection/behaviors/mbbDarknetYOLO.h"
#include "rrlib/coviroa/tImage.h"
#include <vector>
#include "plugins/ib2c/tPercept.h"
#include <opencv2/core.hpp>



#include "rrlib/machine_learning_appliance/ml_definitions.h"
#include <darknet.h>


//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
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

    enum class tDarknetUnimogClass
{
  eSTOP = 0,
  eSIGN1,
  eSIGN2,
  eCONES,
  eUNIMOG
};
using tDarknetDetection = rrlib::machine_learning_appliance::tMLDetection2D<tDarknetUnimogClass>;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * this uses tinyYolo v4 to detect stop signs and stuff
 */
class mImageDetectorYolo : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tInput<std::vector<rrlib::coviroa::tImage>> in_images;
  tOutput<rrlib::coviroa::tImage> out_image; //for the classifier
  
//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mImageDetectorYolo(core::tFrameworkElement *parent, const std::string &name = "ImageDetectorYolo");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  virtual ~mImageDetectorYolo();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:
    
  network * nw_;
  layer l_;
  float nms_ = 0.4;
  image **alphabet_;
  float threshold = 0.3f;
  const std::string unimog_config = "/home/agrosy/Documents/darknet/yolov4-tiny.cfg";
  const std::string unimog_weights = "/home/agrosy/Documents/darknet/yolov4-tiny_last.weights";
  int classCount = 5;
  detection *dets = nullptr;
  

  virtual void OnStaticParameterChange() override;  

  virtual void OnParameterChange() override;  
  virtual void Update() override;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
