/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <vector>
#include <list>

#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include "cv_toolbox.h"
#include "pnp_solver.h"

namespace LCP{

#define SCALE 5             // resize scale for svm
//#define CREATE_SVM_SAMPLE

/**
 *  This class describes the armor information, including maximum bounding box, vertex, standard deviation.
 */
struct ArmorInfo {
  cv::RotatedRect rect;
  int8_t id;
  cv::Point3f pt;
  
  ArmorInfo(cv::RotatedRect r): rect(r), id(-1){ };
  ArmorInfo(): id(-1){ };
};

/**
 * @brief This class achieved functions that can help to detect armors of RoboMaster vehicle.
 */
class ConstraintSet {
 public:
  ConstraintSet(std::string camera_type_);

  void Detect(uint8_t& , std::vector<ArmorInfo>& );
  
  void DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights);
 
  void FilterLights(std::vector<cv::RotatedRect> &lights);
 
  void PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors);
  
  void FilterArmors(std::vector<ArmorInfo> &armors);
  void FilterArmors_svm(std::vector<ArmorInfo> &armors);
  
  ~ConstraintSet();

  cv::Mat src_img_;   // 用于调试
  cv::Mat gray_img_;  // 用于调试

  cv::Ptr<cv::ml::SVM> svm_;

  PnpSolver pnp_solver_big;
  PnpSolver pnp_solver_small;

  cv::HOGDescriptor hog_descriptor_;

  std::vector<ArmorInfo> filter_rects;

  void PrintArmorInfo(const ArmorInfo & armor)
  {
    ROS_INFO("==ArmorInfo==");
    ROS_INFO("id: %d",armor.id);
    ROS_INFO("target_2d_x: %f",armor.rect.center.x);
    ROS_INFO("target_2d_y: %f\n",armor.rect.center.y);
    ROS_INFO("target_3d_x: %f",armor.pt.x);
    ROS_INFO("target_3d_y: %f",armor.pt.y);
    ROS_INFO("target_3d_z: %f\n",armor.pt.z);
  }
  void adjustRect(cv:: RotatedRect & rect)
  {
    if(rect.size.width > rect.size.height)
    {
      double temp = rect.size.height;
      rect.size.height = rect.size.width;
      rect.size.width = temp;
      rect.angle += 90;
      if(rect.angle > 180)
        rect.angle -= 180;
    }
      
    if(rect.angle > 90)
        rect.angle -= 90;
    else if(rect.angle < -90)
        rect.angle += 90;   // 左灯条角度为负, 右灯条角度为正
  }
  
private:

  std::shared_ptr<CVToolbox> cv_toolbox_;

  //! Use for debug
  cv::Mat show_lights_;
  cv::Mat show_armors_befor_filter_;
  cv::Mat show_armors_after_filter_;

  // image threshold parameters
	float light_threshold_;
  float blue_threshold_;
  float red_threshold_;

  // light threshold parameters
  float light_min_area_;
	float light_max_area_;
	float light_min_angle_;
  float light_max_angle_;
	float light_min_angle_diff_;
	float light_max_angle_diff_;
	float light_min_aspect_ratio_;
	float light_max_aspect_ratio_;

	// armor threshold parameters
	float light_max_width_diff_;
	float light_max_height_diff_;
  float armor_min_area_;
	float armor_max_area_;
	float armor_min_angle_;
  float armor_max_angle_;
  float armor_light_angle_diff_;
	float armor_min_ratio_;
	float armor_max_ratio_;
	float armor_min_aspect_ratio_;
	float armor_max_aspect_ratio_;
	float filter_armor_area_;

  cv::Mat color_diff_img_;       // subtract
  cv::Mat binary_gray_img_;      // 亮度二值化
  cv::Mat binary_color_img_;     // 颜色二值化
  cv::Mat color2gray_img_;       // &
  //std::vector<std::vector<cv::Point>> contours_light;
  //std::vector<std::vector<cv::Point>> contours_brightness;
  std::vector<cv::RotatedRect> light_rects;
};

} // namespace LCP