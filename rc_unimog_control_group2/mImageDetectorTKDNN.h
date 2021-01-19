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
/*!\file    projects/finroc_projects_robprak2020_2/mImageDetectorTKDNN.h
 *
 * \author  Manuel Vogel
 *
 * \date    2021-01-15
 *
 * \brief Contains mImageDetectorTKDNN
 *
 * \b mImageDetectorTKDNN
 *
 * image detection using tkdnn
 * image detection using tkdnn
 * image detection using tkdnn
 * image detection using tkdnn
 * image detection using tkdnn
 * image detection using tkdnn
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__finroc_projects_robprak2020_2__mImageDetectorTKDNN_h__
#define __projects__finroc_projects_robprak2020_2__mImageDetectorTKDNN_h__

#include "plugins/structure/tModule.h"

#include "rrlib/geometry/tBoundingBox.h"
#include "rrlib/machine_learning_appliance/tMLDetection.h"
#include "rrlib/coviroa/tImage.h"
#include <vector>
#include "plugins/ib2c/tPercept.h"
#include <opencv2/core.hpp>


#include <tkDNN/tkdnn.h>
#include <tkDNN/Yolo3Detection.h>
#include "rrlib/machine_learning_appliance/ml_definitions.h"

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

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * image detection using tkdnn
 * image detection using tkdnn
 * image detection using tkdnn
 * image detection using tkdnn
 * image detection using tkdnn
 * image detection using tkdnn
 */
class mImageDetectorTKDNN : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tInput<std::vector<rrlib::coviroa::tImage>> in_images;
  tOutput<rrlib::coviroa::tImage> out_image;
  tOutput<bool> stop;
  tOutput<bool> give_way;
  tOutput<bool> cones;
  tOutput<bool> right_of_way;
  tOutput<bool> unimog;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mImageDetectorTKDNN(core::tFrameworkElement *parent, const std::string &name = "ImageDetectorTKDNN");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  virtual ~mImageDetectorTKDNN();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tk::dnn::DetectionNN *detNN; 
  
  std::vector<tk::dnn::box> detected_bbox;
  
  double conf_thresh = 0.6;
  
  int n_batch =1;
  
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
