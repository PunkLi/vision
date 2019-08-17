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

#include "constraint_set.h"
#include <stdio.h>
namespace LCP{

#define ENEMY_IS_RED      // 识别红色
//#define SHOW_DEBUG_IMG
//#define CREATE_SVM_SAMPLE
//#define USE_SVM

ConstraintSet::ConstraintSet(std::string camera_type_): pnp_solver_small(12.6, 12.6), pnp_solver_big(21.6, 12.6)
{
  hog_descriptor_.winSize = cv::Size(64,64);

  std::string file_name = ros::package::getPath("roborts_vision") + "/module/detect_factory/armor/LCP/config/" + camera_type_ + "_param.xml";
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  if(!fs.isOpened())
      ROS_ERROR ("Cannot open armor param file for ConstraintSet, please check if the file is exist.");

  // image threshold parameters
  fs["light_threshold"] >> light_threshold_;
  fs["blue_threshold"] >> blue_threshold_;
  fs["red_threshold"] >> red_threshold_;

  // light threshold parameters
  fs["light_min_area"] >> light_min_area_;
  fs["light_max_area"] >> light_max_area_;
  fs["light_min_angle"] >> light_min_angle_;
  fs["light_max_angle"] >> light_max_angle_;  
  fs["light_min_angle_diff"] >> light_min_angle_diff_; 
  fs["light_max_angle_diff"] >> light_max_angle_diff_;
  fs["light_min_aspect_ratio"] >> light_min_aspect_ratio_;
  fs["light_max_aspect_ratio"] >> light_max_aspect_ratio_;

  // armor threshold parameters
  fs["light_max_width_diff"] >> light_max_width_diff_;
  fs["light_max_height_diff"] >> light_max_height_diff_;
  fs["armor_min_area"] >> armor_min_area_;
  fs["armor_max_area"] >> armor_max_area_;
  fs["armor_min_angle"] >> armor_min_angle_;
  fs["armor_max_angle"] >> armor_max_angle_;
  fs["armor_light_angle_diff"] >> armor_light_angle_diff_;
  fs["armor_min_ratio"] >> armor_min_ratio_;
  fs["armor_max_ratio"] >> armor_max_ratio_;
  fs["armor_min_aspect_ratio"] >> armor_min_aspect_ratio_;
  fs["armor_max_aspect_ratio"] >> armor_max_aspect_ratio_;
  fs["filter_armor_area"] >> filter_armor_area_;

  std::string svm_file = ros::package::getPath("roborts_vision") + "/module/detect_factory/armor/LCP/config/armor_model.yml";
  svm_ = cv::ml::StatModel::load<cv::ml::SVM>(svm_file);
}

void ConstraintSet::Detect(uint8_t &vision_data_status, std::vector<ArmorInfo>& armors) 
{	
  std::vector<cv::RotatedRect> lights;

#ifdef SHOW_DEBUG_IMG
  show_lights_  = cv::Mat::zeros(src_img_.size(), CV_8UC3);
  show_armors_befor_filter_  = src_img_.clone();
  show_armors_after_filter_  = src_img_.clone();
  cv::waitKey(1);
#endif

  DetectLights(src_img_, lights);  // cost: 2.6ms ~ 6ms
	//ROS_ERROR("detect lights number: %d",lights.size());
  PossibleArmors(lights, armors);
#ifdef USE_SVM
  FilterArmors_svm(armors);
#else
  FilterArmors(armors);
#endif
}

void ConstraintSet::DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights) 
{
	auto speed_test_start_begin_time = std::chrono::steady_clock::now();
	
  // step1 threshold gray
	cv::cvtColor(src, gray_img_, cv::ColorConversionCodes::COLOR_BGR2GRAY);
  cv::threshold(gray_img_, binary_gray_img_, light_threshold_, 255, CV_THRESH_BINARY);

	// step2 threshold color
	std::vector<cv::Mat> bgr_channel;
	cv::split(src, bgr_channel);
#ifdef ENEMY_IS_RED
	cv::subtract(bgr_channel[2], bgr_channel[1], color_diff_img_);
	cv::threshold(color_diff_img_, binary_color_img_, red_threshold_, 255, CV_THRESH_BINARY);
#else
	cv::subtract(bgr_channel[0], bgr_channel[2], color_diff_img_);
	cv::threshold(color_diff_img_, binary_color_img_, blue_threshold_, 255, CV_THRESH_BINARY);
#endif

	//step3 gray & color
  //cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  //cv::dilate(binary_color_img_, binary_color_img_, element, cv::Point(-1, -1), 1);
  //cv::morphologyEx(binary_color_img_,binary_color_img_, MORPH_OPEN, element);
  color2gray_img_ = binary_color_img_ & binary_gray_img_;

	//step4 contours
  std::vector<std::vector<cv::Point>> contours_light;
	cv::findContours(color2gray_img_, contours_light, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  std::vector<std::vector<cv::Point>> contours_brightness;
	cv::findContours(binary_gray_img_, contours_brightness, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  
  auto cost1 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count() / 1000.0;
  //ROS_INFO("line: %d: time cost = %.2f ms", __LINE__, cost1);


  lights.reserve(contours_brightness.size());
  std::vector<bool> is_processes(contours_brightness.size());
  
  //ROS_WARN("contours_light.size(): %d", contours_light.size());
  //ROS_WARN("contours_brightness.size(): %d", contours_brightness.size());

  speed_test_start_begin_time = std::chrono::steady_clock::now();
  
//#pragma omp parallel
  for (unsigned int i = 0; i < contours_light.size(); ++i) {
//#pragma omp for
    for (unsigned int j = 0; j < contours_brightness.size(); ++j) {
      if (!is_processes[j]) {
        if (cv::pointPolygonTest(contours_brightness[j], contours_light[i][0], true) >= 0.0) 
        {
          cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[j]);
         // if (lights[i].size.area() > light_min_area_ && lights[i].size.area() < light_max_area_)
          //{
            //cv_toolbox_->adjustRect(single_light); 
            lights.push_back(single_light); 
            cv_toolbox_->DrawRotatedRect(show_lights_, single_light, cv::Scalar(0,250,0), 1);
            is_processes[j] = true;
            break;
         // } // area threshold
        } // pointPoltgonTest
      } // is_processes
    } // for j loop
  } // for i loop
  //ROS_WARN("after size: %d", lights.size());

  auto cost = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count() / 1000.0;
  //ROS_INFO("line: %d: Detetc & Filter Lights time cost = %.2f ms", __LINE__, cost);
  
  
  std::vector<cv::RotatedRect> lights_filter;
//#pragma omp parallel for 
  for(int i = 0; i < lights.size(); ++i){
  	this->adjustRect(lights[i]); 	
  }

  for (int i = 0; i < lights.size(); ++i)
	{
    if (lights[i].size.area() > light_min_area_ && lights[i].size.area() < light_max_area_)
    {
      //auto rect = std::minmax(single_light.size.width, single_light.size.height);
      //auto light_aspect_ratio = rect.second / rect.first;
      if (abs(lights[i].angle) < light_max_angle_) 
      {
        lights_filter.push_back(lights[i]); // 高速水平移动的灯条
        cv_toolbox_->DrawRotatedRect(show_lights_, lights[i], cv::Scalar(255,255,0), 1);
      }
    }
  }
  lights = lights_filter;

#ifdef SHOW_DEBUG_IMG
	//cv::imshow("gray_img", gray_img_);
  //cv::imshow("binary_color_img_", binary_color_img_);
  //cv::imshow("color2gray_img_", color2gray_img_);
	//cv::imshow("binary_gray_img_", binary_gray_img_);
  //cv::imshow("show_lights_", show_lights_);
#endif
}

void ConstraintSet::PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armor_vector) {
  
  auto speed_test_start_begin_time = std::chrono::steady_clock::now();
  for (int i = 0; i < lights.size(); ++i) {
    for (int j = i; j < lights.size(); ++j) {
      if (i==j) continue;
	    //auto rect1 = std::minmax(lights[i].size.width, lights[i].size.height);
      //auto light_aspect_ratio1 = rect1.second / rect1.first;
	    //auto rect2 = std::minmax(lights[j].size.width, lights[j].size.height);
      //auto light_aspect_ratio2 = rect2.second / rect2.first;
      //auto long_diff = lights[i].size.area() / lights[j].size.area();
	    double angle_diff = lights[i].angle - lights[j].angle;

	    double height_diff_ratio = abs(lights[i].size.height - lights[j].size.height) / 
                                 std::max(lights[i].size.height, lights[j].size.height);

	    //auto width_diff  = abs(lights[i].size.width - lights[j].size.width) / std::max(lights[i].size.width, lights[j].size.width);

	    if (abs(angle_diff) < 10 && height_diff_ratio < 0.5)	
	    {
		    cv::RotatedRect possible_rect;
		    if (lights[i].center.x < lights[j].center.x)
		      possible_rect = cv_toolbox_->boundingRRect(lights[i],lights[j]);
		    else
		      possible_rect = cv_toolbox_->boundingRRect(lights[j],lights[i]);
				
		    double armor_ratio = possible_rect.size.width / possible_rect.size.height;
		    double armor_angle = possible_rect.angle;
		    //auto armor_area = possible_rect.size.area();
		    double armor_light_angle_diff = (double)abs(armor_angle - lights[i].angle) + (double)abs(armor_angle - lights[j].angle); // 左右灯条的积累差
		
		    //ROS_INFO("angle_1\t%f",lights[i].angle);
		    //ROS_INFO("angle_2\t%f",lights[j].angle);
		    //ROS_INFO("angle_diff\t%f",angle_diff);

		    //ROS_INFO("height_diff\t%f", height_diff);
		    //ROS_INFO("width_diff\t%f\n",width_diff);
		    //ROS_INFO("armor_ratio\t%f",armor_ratio);
		    //ROS_INFO("armor_angle\t%f",armor_angle);
		    
		    if (0.8 < armor_ratio && armor_ratio < 3 && 
            armor_angle < 30 && 
            armor_light_angle_diff < 20)
		    {	
          //ROS_INFO("armor_light_angle_diff: %f\n", armor_light_angle_diff);
    
			    ArmorInfo armor(possible_rect);
			    armor_vector.push_back(armor);
		    } // get_armor
	    }// 2个严格平行
	  } // for j loop
  } // for i loop

  for (int i = 0; i != armor_vector.size(); ++i)
    cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, armor_vector[i].rect, cv::Scalar(0,255,0), 2);
  //cv::imshow("armors_before_filter", show_armors_befor_filter_);
  
  auto cost = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count() / 1000.0;
  //ROS_INFO("line: %d: Possible Armor time cost = %.2f ms", __LINE__, cost);
}


bool makeRectSafe(cv::Rect & rect, cv::Size size){
    if (rect.x < 0)
        rect.x = 0;
    if (rect.x + rect.width > size.width)
        rect.width = size.width - rect.x;
    if (rect.y < 0)
        rect.y = 0;
    if (rect.y + rect.height > size.height)
        rect.height = size.height - rect.y;
    if (rect.width <= 0 || rect.height <= 0)
        return false;
    return true;
}

int img_idx = 1;
    cv::Mat img;
cv::Mat armor_roi ;

int armorToarmorTest(const cv::RotatedRect & _rect1, const cv::RotatedRect & _rect2)
{
	cv::Point2f center1 = _rect1.center;
	cv::Point2f center2 = _rect2.center;
	cv::Rect rect1 = _rect1.boundingRect();
	cv::Rect rect2 = _rect2.boundingRect();

	if (rect1.x < center2.x && center2.x < rect1.x + rect1.width  && 
		  rect2.x < center1.x && center1.x < rect2.x + rect2.width  && 
		  rect1.y < center2.y && center2.y < rect1.y + rect1.height &&
		  rect2.y < center1.y && center1.y < rect2.y + rect2.height )
		{
			if(_rect1.size.area() > _rect2.size.area()) return 1;
			else return 2;
		}
	return -1;
}

void DrawArmor(const cv::Mat &img, ArmorInfo &armor, const cv::Scalar &color, int thickness) {
    cv::Point2f vertex[4];

    cv::Point2f center = armor.rect.center;
    std::ostringstream ss;
    ss << (int)(armor.id);
    std::string text(ss.str());
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 1;
    cv::putText(img, text, center, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);

    armor.rect.points(vertex);
    for (int i = 0; i < 4; i++)
      cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
  }

void ConstraintSet::FilterArmors_svm(std::vector<ArmorInfo> &armors) {
  
  auto speed_test_start_begin_time = std::chrono::steady_clock::now();

#ifndef CREATE_SVM_SAMPLE
  std::vector<bool> is_armor(armors.size(), true);

  for (int i = 0; i < armors.size(); i++) {
	  	for (int j = i + 1; j < armors.size(); j++) {
			int flag = armorToarmorTest(armors[i].rect, armors[j].rect); // armor严重重合的筛选掉
			if(flag == 1) is_armor[i] = false;
			else if(flag == 2) is_armor[j] = false;
		}
	}

  double dis;
#pragma omp parallel
  for (int i = 0; i < armors.size(); i++) 
  {
#pragma omp for
	  for (int j = i + 1; j < armors.size(); j++) //  && (is_armor[j] == true)
		{
      if (is_armor[i] == true && is_armor[j] == true)
      {
        dis = POINT_DIST(armors.at(i).rect.center, armors.at(j).rect.center);
        
        if (dis <= std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height)+ 
                   std::max(armors.at(j).rect.size.width, armors.at(j).rect.size.height) )
        {
          double armor_ratio1 = std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height) / std::min(armors.at(i).rect.size.width, armors.at(i).rect.size.height); 
          double armor_ratio2 = std::max(armors.at(j).rect.size.width, armors.at(j).rect.size.height) / std::min(armors.at(j).rect.size.width, armors.at(j).rect.size.height); 

          /* if (abs(armor_ratio1 - armor_ratio2) > 0.4) // 这个放在新代码里面太苛刻了
					{
            if(armor_ratio1 - armor_ratio2 > 0)
              is_armor[i] = false;
            else
              is_armor[j] = false; 
				  }*/
					if (abs(abs(armors.at(i).rect.angle) - abs(armors.at(j).rect.angle)) > 15) 
					{
						if (abs(armors.at(i).rect.angle) > abs(armors.at(j).rect.angle))
              is_armor[i] = false;
            else
              is_armor[j] = false;
					}
          else if (armors.at(i).rect.size.area() - armors.at(j).rect.size.area() > 100)
              is_armor[i] = false;
				}  // dis < dist
		 	}   // if both
    }	   // for j
  }	    // for i
#endif
  cv::resize(gray_img_, gray_img_, cv::Size(1600 / SCALE ,900 / SCALE));

  for(int i = 0; i < armors.size(); ++i)
  {
#ifndef CREATE_SVM_SAMPLE
    if(is_armor[i] == true)
    {
#endif
      auto rect = armors[i].rect;
      auto center = rect.center / SCALE;

      warpAffine(gray_img_, img, 
                cv::getRotationMatrix2D(center, rect.angle, 1), 
                img.size(), 
                cv::INTER_LINEAR, 
                cv::BORDER_CONSTANT);

      cv::Rect roi = cv::Rect(center.x - (rect.size.width  / 2) / SCALE, 
                              center.y - (rect.size.height / 2) / SCALE, 
                              rect.size.width / SCALE, 
                              rect.size.height / SCALE);

      if (makeRectSafe(roi, img.size()) == true)
      {
        armor_roi = img(roi);
        cv::resize(armor_roi, armor_roi, cv::Size(64,64));
        cv::imshow("armor_roi", armor_roi);

#ifdef CREATE_SVM_SAMPLE
        char str[100];
        sprintf(str, "../data/sample2/armor_%d.jpg", img_idx++);
        cv::imwrite(str, armor_roi);
#else
        std::vector<float> feature;
        hog_descriptor_.compute( armor_roi, feature, cv::Size(8,8), cv::Size(0,0)); 
        armors[i].id = svm_->predict(feature);
#endif
      }
#ifndef CREATE_SVM_SAMPLE
    }
#endif
  }
#ifndef CREATE_SVM_SAMPLE
	filter_rects.clear();
	for( int i = 0; i < is_armor.size();++i)
		if(is_armor[i] && 0 < armors[i].id && armors[i].id < 9) // svm 1~8
      filter_rects.push_back(armors[i]);
	armors = filter_rects;

  for (int i = 0; i != armors.size(); ++i)
  {
    if (armors[i].id == 1 || armors[i].id == 7 || armors[i].id == 8)
      pnp_solver_big.GetXYZ(armors[i].rect, armors[i].pt);
    else
      pnp_solver_small.GetXYZ(armors[i].rect, armors[i].pt);
    //PrintArmorInfo(armors[i]);

    DrawArmor(src_img_, armors[i], cv::Scalar(0,255,0), 2);
  }
#endif
  //cv::imshow("armors_after_filter", src_img_);
  
  auto cost = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count() / 1000.0;
  //ROS_INFO("line: %d: FilterArmors_svm time cost = %.2f ms", __LINE__, cost);
}

void ConstraintSet::FilterArmors(std::vector<ArmorInfo> &armors) {
  
  auto speed_test_start_begin_time = std::chrono::steady_clock::now();

  std::vector<bool> is_armor(armors.size(), true);

  for (int i = 0; i < armors.size(); i++) {
	  	for (int j = i + 1; j < armors.size(); j++) {
			int flag = armorToarmorTest(armors[i].rect, armors[j].rect); // armor严重重合的筛选掉
			if(flag == 1) is_armor[i] = false;
			else if(flag == 2) is_armor[j] = false;
		}
	}

  double dis;
#pragma omp parallel
  for (int i = 0; i < armors.size(); i++) 
  {
#pragma omp for
	  for (int j = i + 1; j < armors.size(); j++) //  && (is_armor[j] == true)
		{
      if (is_armor[i] == true && is_armor[j] == true)
      {
        dis = POINT_DIST(armors.at(i).rect.center, armors.at(j).rect.center);
        
        if (dis <= std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height)+ 
                   std::max(armors.at(j).rect.size.width, armors.at(j).rect.size.height) )
        {
          double armor_ratio1 = std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height) / std::min(armors.at(i).rect.size.width, armors.at(i).rect.size.height); 
          double armor_ratio2 = std::max(armors.at(j).rect.size.width, armors.at(j).rect.size.height) / std::min(armors.at(j).rect.size.width, armors.at(j).rect.size.height); 

          /* if (abs(armor_ratio1 - armor_ratio2) > 0.4) // 这个放在新代码里面太苛刻了
					{
            if(armor_ratio1 - armor_ratio2 > 0)
              is_armor[i] = false;
            else
              is_armor[j] = false; 
				  }*/
					if (abs(abs(armors.at(i).rect.angle) - abs(armors.at(j).rect.angle)) > 15) 
					{
						if (abs(armors.at(i).rect.angle) > abs(armors.at(j).rect.angle))
              is_armor[i] = false;
            else
              is_armor[j] = false;
					}
          else if (armors.at(i).rect.size.area() - armors.at(j).rect.size.area() > 100)
              is_armor[i] = false;
				}  // dis < dist
		 	}   // if both
    }	   // for j
  }	    // for i

	filter_rects.clear();
	for( int i = 0; i < is_armor.size();++i) filter_rects.push_back(armors[i]);
	armors = filter_rects;

  for (int i = 0; i != armors.size(); ++i)
  {
    armors[i].id = 3;
    pnp_solver_small.GetXYZ(armors[i].rect, armors[i].pt);
    //PrintArmorInfo(armors[i]);
    DrawArmor(src_img_, armors[i], cv::Scalar(0,255,0), 2);
  }
  //cv::imshow("armors_after_filter", src_img_);
  
  auto cost = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count() / 1000.0;
  //ROS_INFO("line: %d: FilterArmors time cost = %.2f ms", __LINE__, cost);
}

ConstraintSet::~ConstraintSet() {

}

} // namespace LCP