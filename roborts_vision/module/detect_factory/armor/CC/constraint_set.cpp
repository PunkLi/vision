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

namespace CC{

//#define SHOW_DEBUG_IMG
//#define enemy_red
// 方便输出一些调试信息
#define LINE(str)  //std::cout << "Code Line:" << __LINE__ << "\t" << str << std::endl;
#define LINE_INFO(str,str2) //std::cout << "Code Line:" << __LINE__ << "\t" << str <<":\t"<< str2 << std::endl;
#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))
ConstraintSet::ConstraintSet()
{ svm_big = StatModel::load<SVM>(ros::package::getPath("roborts_vision") + "/module/detect_factory/armor/CC/big_armor_model.yml");
  svm_small = StatModel::load<SVM>(ros::package::getPath("roborts_vision") + "/module/detect_factory/armor/CC/armor_model.yml");
  std::string file_name = ros::package::getPath("roborts_vision") + "/module/detect_factory/armor/CC/constraint_set.xml";
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  if(!fs.isOpened())
      ROS_ERROR ("Cannot open armor param file, please check if the file is exist");
  
  // algorithm info
  fs["enable_debug"] >> enable_debug_;
  fs["enemy_color"] >> enemy_color_;
  fs["using_hsv"] >> using_hsv_;

	// image threshold parameters 
  #ifndef enemy_red
  fs["light_threshold_for_blue"] >> light_threshold_;
  #else
  fs["light_threshold"] >> light_threshold_;
  #endif
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
}
float ConstraintSet::small_armor_svm(cv::Mat& img_roi)
{
	// speed_test_reset();
    cv::Mat gradient_lst;
    cv::HOGDescriptor hog;
	cv::Size wsize =  cv::Size(60,25);

	cv::resize(img_roi, img_roi, cv::Size(60,25));
    hog.winSize = wsize / 8 * 8;
    std::vector< float > descriptors;
    
    cv::Rect roi = cv::Rect((img_roi.cols - wsize.width ) / 2,
                    (img_roi.rows - wsize.height ) / 2,
                   	 wsize.width,
                  	 wsize.height);
    hog.compute( img_roi(roi), descriptors, cv::Size(8,8), cv::Size(0,0));   
	float response = svm_small->predict(descriptors);
	// speed_test_end("armor_svm 用时 = ", "ms");
	return response;
}

float ConstraintSet::armor_svm(cv::Mat& img_roi)
{
	// speed_test_reset();
    cv::Mat gradient_lst;
    cv::HOGDescriptor hog;
	cv::Size wsize =  cv::Size(100,25);
	cv::resize(img_roi, img_roi, cv::Size(100,25));
    hog.winSize = wsize / 8 * 8;
    std::vector< float > descriptors;
    
    cv::Rect roi = cv::Rect((img_roi.cols - wsize.width ) / 2,
                    (img_roi.rows - wsize.height ) / 2,
                   	 wsize.width,
                  	 wsize.height);
    hog.compute( img_roi(roi), descriptors, cv::Size(8,8), cv::Size(0,0));   
	float response = svm_big->predict(descriptors);
	// speed_test_end("armor_svm 用时 = ", "ms");
	return response;
}

float ConstraintSet::get_armor_roi(cv::RotatedRect& rect, bool visual)
{
    //float val;
    cv::Mat frontImg;
	float val;
	float p=std::max(float(rect.size.height/rect.size.width),float(rect.size.width/rect.size.height));
	cv::Point2f ve[4];
    rect.points(ve);
    cv::Point2f tl=ve[1],tr=ve[2],dl=ve[0],dr=ve[3];
	cv::Point2f src[4]{cv::Vec2f(tl), cv::Vec2f(tr), cv::Vec2f(dr), cv::Vec2f(dl)};
	if(p < 3.0){
			cv::Point2f dst[4]{cv::Point2f(0.0, 0.0), cv::Point2f(60, 0.0), cv::Point2f(60, 25),cv::Point2f(0.0, 25) };
	const cv::Mat perspMat = getPerspectiveTransform(src, dst);
	cv::warpPerspective(gray_img_, frontImg, perspMat, cv::Size(60, 25));
	val = small_armor_svm(frontImg);}
	else {
			cv::Point2f dst[4]{cv::Point2f(0.0, 0.0), cv::Point2f(100, 0.0), cv::Point2f(100, 25), cv::Point2f(0.0, 25)};
	const cv::Mat perspMat = getPerspectiveTransform(src, dst);
	cv::warpPerspective(gray_img_, frontImg, perspMat, cv::Size(100, 25));
	val = armor_svm(frontImg);}
    // char str[100];
	// 			sprintf(str, "%.2f", 
	// 			val);
	// 			cv::putText(frontImg, str, cv::Point(5, 20), CV_FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, CV_RGB(100,100,100), 1);
	// //if(visual)
	// //{
	// 	cv::imshow("svm_mat",frontImg);
	//}
	/*auto center = rect.center;
	cv::Mat rot_mat = cv::getRotationMatrix2D(rect.center, rect.angle, 1); 
	cv::Mat img;
	warpAffine(gray_img_, img, rot_mat, img.size(), INTER_LINEAR, BORDER_CONSTANT); // warpAffine use 2ms
	// cv::imshow("warpaffine", img);
	cv::Rect roi = cv::Rect(center.x - (rect.size.width / 2), 
						    center.y - (rect.size.height / 2), 
					  	    rect.size.width, rect.size.height);
	if (makeRectSafe(roi, img.size()) == true)
	{Mat frontImg;
		cv::Mat armor_roi = img(roi);
		if(visual) cv::imshow("armor_roi", armor_roi);

		float wh = roi.width, gh = roi.height;
		float ratio = wh/gh;

		// 这里简单地根据装甲片的长宽比例判定了大小装甲，然后最终用的是SVM分类器
		
		if(ratio < 3){
			
			val = small_armor_svm(armor_roi);}
		else
			{val = armor_svm(armor_roi);
	}
		// std::cout << "svm lebel:" << val << std::endl;
		return val;
	}*/
	return val;
}
void ConstraintSet::Detect(uint8_t &vision_data_status, std::vector<ArmorInfo>& armors) 
{	
  std::vector<cv::RotatedRect> lights;

  cv::cvtColor(src_img_, gray_img_, CV_BGR2GRAY);
  if (enable_debug_) {
      show_lights_before_filter_ = cv::Mat::zeros(src_img_.size(), CV_8UC3);
  	  show_lights_after_filter_  = cv::Mat::zeros(src_img_.size(), CV_8UC3);
  	  show_armors_befor_filter_  = src_img_.clone();
  	  show_armors_after_filter_  = src_img_.clone();
      cv::waitKey(1);
    }

    DetectLights(src_img_, lights);
		//ROS_INFO("detect lights number: %d", lights.size());
		if(lights.size()>200) return;

    FilterLights(lights);
    PossibleArmors(lights, armors);
    FilterArmors(armors);
		
    if(!armors.empty()) {
      vision_data_status = 1;
    } else
      vision_data_status = 0;
}

void ConstraintSet::DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights) 
{
	cv::Mat subtract_color_img;
	cv::Mat binary_brightness_img;
    cv::Mat binary_color_img;//(1280,720,CV_8UC(1),cv::Scalar(0));
    cv::Mat binary_light_img;
    cv::Mat binary_color_img_red1;
	cv::Mat binary_color_img_red2;
    cv::Mat binary_brightness_img_halo;
    cv::Mat img_hsv,img_hsv1;
    
      //cv::cvtColor(src, img_hsv, CV_BGR2HSV);
	//   #ifdef enemy_red
	//   std::vector<cv::Mat> bgr_channel;
	//   cv::split(src, bgr_channel);
	//   cv::subtract(bgr_channel[2], bgr_channel[1], subtract_color_img);
	//    float thresh = red_threshold_;
	//   cv::threshold(subtract_color_img, binary_color_img, thresh, 255, CV_THRESH_BINARY);
	//   cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	//   cv::dilate(binary_color_img, binary_color_img, element, cv::Point(-1, -1), 1);
	//   #else 
	//   std::vector<cv::Mat> bgr_channel;
	//   cv::split(src, bgr_channel);
	//   cv::subtract(bgr_channel[0], bgr_channel[1], subtract_color_img);
	//    float thresh = blue_threshold_;
	//   cv::threshold(subtract_color_img, binary_color_img, thresh, 255, CV_THRESH_BINARY);
	//  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	//   cv::dilate(binary_color_img, binary_color_img, element, cv::Point(-1, -1), 1);
	//   #endif
      #ifndef enemy_red
	   cv::cvtColor(src, img_hsv, CV_BGR2HSV);
        // cv::Mat img_hsv_blue, img_threshold_blue;
        //img_hsv_blue = img_hsv.clone();
        cv::Mat blue_low(cv::Scalar(90, 130, 130));// gx 90 130 130
        cv::Mat blue_higher(cv::Scalar(140, 255, 255));
	   // cv::Mat blue_low(cv::Scalar(0, 100, 100));
       // cv::Mat blue_higher(cv::Scalar(25, 255, 255));
        cv::inRange(img_hsv, blue_low, blue_higher, binary_color_img);
    #else
        cv::cvtColor(src, img_hsv, CV_BGR2HSV);
	    cv::Mat blue_low(cv::Scalar(0, 100, 100));
         cv::Mat blue_higher(cv::Scalar(30, 255, 255));
	 //cv::Mat blue_low(cv::Scalar(150, 100, 100));
        //cv::Mat blue_higher(cv::Scalar(180, 255, 255));
	   // cv::Mat blue_low(cv::Scalar(0, 100, 100));
       // cv::Mat blue_higher(cv::Scalar(10, 255, 255));
        cv::inRange(img_hsv, blue_low, blue_higher, binary_color_img);
    
	 //  cv::Mat blue_low(cv::Scalar(0, 100, 100));
     //   cv::Mat blue_higher(cv::Scalar(20, 255, 255));
	   // cv::Mat blue_low(cv::Scalar(0, 100, 100));
       // cv::Mat blue_higher(cv::Scalar(10, 255, 255));
     //   cv::inRange(img_hsv, blue_low, blue_higher, binary_color_img);
	   // cv::Mat red_low1(cv::Scalar(0, 100, 100));
       // cv::Mat red_higher1(cv::Scalar(10, 255, 255));
       // cv::inRange(img_hsv, red_low1, red_higher1, binary_color_img_red1);
	 //   cv::Mat red_low2(cv::Scalar(160, 100, 100));
     //   cv::Mat red_higher2(cv::Scalar(180, 255, 255));
     //   cv::inRange(img_hsv, red_low2, red_higher2, binary_color_img_red2);
	 //   binary_color_img=binary_color_img_red2;//&&binary_color_img;
	   // cv::Mat subtract_color_img;
	  //  std::vector<cv::Mat> hsv_channel;
	  //  cv::split(img_hsv, hsv_channel);
	    //cv::subtract(bgr_channel[2], bgr_channel[1], subtract_color_img);
        
        //float thresh;
      //  thresh = red_threshold_+100;
// 	  cv::Mat image = hsv_channel[2];
//     for (int i=0;i<hsv_channel[2].rows;i++){
// 	  for(int j=0;j<hsv_channel[2].cols;j++){
// 		   if((hsv_channel[0].at<uchar>(i,j)%155>=0&&
// 		   hsv_channel[0].at<uchar>(i,j)%160<=25)//||
// 		 &&(hsv_channel[1].at<uchar>(i,j)>=100&&
// 		   hsv_channel[1].at<uchar>(i,j)<=255)&&
// 		  (hsv_channel[2].at<uchar>(i,j)>=100&&
// 		   hsv_channel[2].at<uchar>(i,j)<=255))
// 		   image.at<uchar>(i,j)=255;
// 		   else
// 		   image.at<uchar>(i,j)=0;
// 	   }
//    }
//   	image.copyTo(binary_color_img);
       // cv::threshold(subtract_color_img, binary_color_img, thresh, 255, CV_THRESH_BINARY);
	   
    #endif   
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::dilate(binary_color_img,binary_color_img,element);
    cv::morphologyEx(binary_color_img,binary_color_img, CV_MOP_CLOSE, element);
	//cv::morphologyEx(binary_color_img,binary_color_img, CV_MOP_CLOSE, element);
	//cv::imshow("gramadd",img_hsv);
	//#ifndef enemy_red
	//light_threshold_+=1;
	//#endif
    //cv::threshold(gray_img_, binary_brightness_img, light_threshold_, 255, CV_THRESH_BINARY);
	#ifndef enemy_red
	cv::threshold(gray_img_, binary_brightness_img, 190, 255, CV_THRESH_BINARY);
	#else 
	cv::threshold(gray_img_, binary_brightness_img, 100, 255, CV_THRESH_BINARY);//5000ex    day 100   //15000ex  dusk 150
	#endif
	cv::threshold(gray_img_, binary_brightness_img_halo, light_threshold_-100, 255, CV_THRESH_BINARY);
	cv::dilate(binary_brightness_img_halo,binary_brightness_img_halo,element);
	 binary_color_img = binary_color_img & binary_brightness_img_halo;
	//cv::imshow("color_binary",binary_color_img);
	//cv::imshow("liangdu",binary_brightness_img);
	if (enable_debug_){
	cv::imshow("color_binary",binary_color_img);
	//cv::imshow("gramadd",img_hsv1);
	cv::imshow("liangdu",binary_brightness_img);}
    binary_light_img = binary_brightness_img;//binary_color_img & binary_brightness_img;
    
 // }
 /*  else {
    cv::Mat subtract_color_img;
	  std::vector<cv::Mat> bgr_channel;
	  cv::split(src, bgr_channel);
	
	  if (enemy_color_ == 0) // red
		  cv::subtract(bgr_channel[2], bgr_channel[1], subtract_color_img);
	  else
		  cv::subtract(bgr_channel[0], bgr_channel[1], subtract_color_img);
		
    cv::threshold(gray_img_, binary_brightness_img, light_threshold_, 255, CV_THRESH_BINARY);
    
    float thresh;
    if (enemy_color_ == 0) // red
      thresh = red_threshold_;
    else
      thresh = blue_threshold_;

    cv::threshold(subtract_color_img, binary_color_img, thresh, 255, CV_THRESH_BINARY);
    
	binary_light_img = binary_color_img & binary_brightness_img;
 }*/
  //binary_light_img = binary_color_img & binary_brightness_img;
  //std::vector<std::vector<cv::Point>> contours_light;
//	cv::findContours(binary_light_img, contours_light, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  //std::vector<std::vector<cv::Point>> contours_brightness;
	//cv::findContours(binary_brightness_img, contours_brightness, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  
  
std::vector<std::vector<cv::Point> > contours_light;
	cv::findContours(binary_light_img, contours_light, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
lights.reserve(contours_light.size());
  //  std::vector<std::vector<cv::Point> > contours_brightness;
//	cv::findContours(binary_brightness_img, contours_brightness, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> > contours_color;
	cv::findContours(binary_color_img, contours_color, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
   
    for (unsigned int i = 0; i < contours_light.size(); ++i) {
        for (unsigned int j = 0; j < contours_color.size(); ++j) {
      	    if (cv::pointPolygonTest(contours_color[j], contours_light[i][0], true) >= 0.0) 
            {
                cv::RotatedRect single_light = cv::minAreaRect(contours_light[i]);
                lights.push_back(single_light);
				 if (enable_debug_)
             cv_toolbox_->DrawRotatedRect(show_lights_before_filter_, single_light, cv::Scalar(0,255,0), 1);
                break;
            }
        } // for j loop
    } // for i loop
//   for (unsigned int i = 0; i < contours_light.size(); ++i) {
//     for (unsigned int j = 0; j < contours_brightness.size(); ++j) {
//       	if (cv::pointPolygonTest(contours_brightness[j], contours_light[i][0], true) >= 0.0) 
//         {
//           cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[j]);
		  
//           lights.push_back(single_light);
//           if (enable_debug_)
//             cv_toolbox_->DrawRotatedRect(show_lights_before_filter_, single_light, cv::Scalar(0,255,0), 1);
//           break;
//         }
//     } // for j loop
//   } // for i loop
 /*  std::vector<cv::RotatedRect> lights1;
for(int i=0;i<lights.size();i++)
  {if(lights[i].size.area()>30&&lights[i].size.area()<300)
  lights1.push_back(lights[i]);}
  lights=lights1;
  lights1.clear();*/
  if (enable_debug_)
    cv::imshow("show_lights_before_filter", show_lights_before_filter_);
}
void ConstraintSet::FilterLights(std::vector<cv::RotatedRect> &lights) 
{
  std::vector<cv::RotatedRect> light_rects;

#pragma omp parallel for 
  for(uchar i = 0; i < lights.size(); i++ ){
  	cv_toolbox_->adjustRect(lights[i]); 	
  }
  
  for (const auto &armor_light : lights)
	{
    auto rect = std::minmax(armor_light.size.width, armor_light.size.height);
  	auto light_aspect_ratio = rect.second / rect.first;
    auto angle = armor_light.angle;

		if ( // 80 <= abs(angle) && abs(angle) <= 90   // 高速水平移动的灯条,带有拖影  // 特殊情况,无论横竖, 旧版本有这一行代码
		      light_aspect_ratio <= 2.5
		   && armor_light.size.area() >= light_min_area_ // 1.0
		   && armor_light.size.area() < light_max_area_) //* src_img_.size().height * src_img_.size().width) // 0.04
		{
			light_rects.push_back(armor_light); // 高速水平移动的灯条
      if (enable_debug_)
		  	cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, armor_light, cv::Scalar(255,0,0), 1);
		}
     // 针对灯条细小的情况, 没有最大比例的判断, 较为理想的灯条
		else if(armor_light.size.area() >= light_min_area_ // 1.0
		   		&& armor_light.size.area() < 100000  //light_max_area_ * src_img_.size().height * src_img_.size().width // 0.04
		   		&& abs(angle) < light_max_angle_ // 与垂直的偏角17.5 , 这里是可以取消/2的,进一步细化
		        && light_aspect_ratio >=2.5
				&&armor_light.size.area() <= light_max_area_)
		{
			light_rects.push_back(armor_light); // 接近于垂直的灯条, 由于阈值不够合理, 细小的灯条
      if (enable_debug_)
			  cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, armor_light, cv::Scalar(0,255,0), 1);
	  }
      // 检测最为平凡的情况
    else if (//light_aspect_ratio < _para.light_max_aspect_ratio  // 6.8
              armor_light.size.area() >= light_min_area_ // 1.0
			     && armor_light.size.area() < light_max_area_  // 0.04
			     && abs(angle) < light_max_angle_) // 与垂直的偏角35 
    {
      light_rects.push_back(armor_light);
      if (enable_debug_)
			  cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, armor_light, cv::Scalar(0,0,255), 1);
		}
  }
  if (enable_debug_)
		cv::imshow("lights_after_filter", show_lights_after_filter_);
	
  lights = light_rects;
}

void ConstraintSet::PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armor_vector) {
    for (int i = 0; i < lights.size(); ++i) {
    for (int j = i; j < lights.size(); ++j) {
			auto rect1 = std::minmax(lights[i].size.width, lights[i].size.height);
    	auto light_aspect_ratio1 = rect1.second / rect1.first;
			auto rect2 = std::minmax(lights[j].size.width, lights[j].size.height);
    	auto light_aspect_ratio2 = rect2.second / rect2.first;

			auto angle_diff  = abs(lights[i].angle - lights[j].angle);
			auto height_diff = abs(lights[i].size.height - lights[j].size.height) / std::max(lights[i].size.height, lights[j].size.height);
			auto width_diff  = abs(lights[i].size.width - lights[j].size.width) / std::max(lights[i].size.width, lights[j].size.width);

// 快速平移的情况 Fast Move
	// 2个严格平行
			if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 2.5
			    && 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 2.5
			    && abs(lights[i].angle) == 90 && abs(lights[j].angle) == 90     // 角度为0
			    && static_cast<int>(abs(angle_diff)) % 180 == 0
			    && height_diff < 0.5 && width_diff < 0.5)	
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i],lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j],lights[i]);
				
				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_ 
					&& armor_ratio < 4.5                // 经验参数
					&& abs(armor_angle) < 20 )          // 经验参数
				{	
					Armor_Twist armor_twist = FAST_MOVE;
					LINE("[快速移动] armor")
					ArmorInfo armor(possible_rect, armor_twist);
					armor_vector.push_back(armor);
				} // get_armor
			}// 2个严格平行
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// 2个当中有一个略不平行
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 2.5
			     	 && 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 2.5
			    	 && ( (abs(lights[i].angle) == 90 && abs(lights[j].angle) > 80) || (abs(lights[i].angle) > 80 && abs(lights[j].angle) == 90))
			    	 && height_diff < 0.5 && width_diff < 0.5)		
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i],lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j],lights[i]);
				
				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				
				// auto armor_light_angle_diff = abs(lights[i].angle) == 90 ? 
				//						 	     abs(armor_angle) + abs(armor_angle - lights[j].angle - 90); // 左右灯条的积累差

				if (armor_area > armor_min_area_
				    && armor_ratio < 4.5                // 经验参数
					&& abs(armor_angle) < 20 )          // 经验参数
				{	
					Armor_Twist armor_twist = FAST_MOVE;
					LINE("[快速移动] armor")
					ArmorInfo armor(possible_rect, armor_twist);
					armor_vector.push_back(armor);
				} // get_armor
			}// 2个当中有一个略不平行
// 快速平移的情况 Fast Move

// 中速平移的情况 Mid Move
		// 2个严格平行
			else if ( ( (light_aspect_ratio1 == 1 && light_aspect_ratio2 <= 1.5) 
					 || (light_aspect_ratio1 <= 1.5 && light_aspect_ratio2 == 1) )   // 其中一个为正方形
					 && static_cast<int>(abs(angle_diff)) % 90 == 0               	 // 角度差为0
					 && static_cast<int>(abs(lights[i].angle)) % 90 == 0          	 // 角度为0 或 90
					 && static_cast<int>(abs(lights[j].angle)) % 90 == 0          	 // 角度为0 或 90
					 && height_diff < 0.5 && width_diff < 0.5)               	     // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_	    // 经验参数
					&& armor_ratio < 4                      // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20 )             // 经验参数
					{
						Armor_Twist armor_twist = MID_MOVE;
						LINE("[中等速度] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
			// 2个严格平行

			// 1个竖着 1个横着
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.3
					 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.3
					 && static_cast<int>(abs(angle_diff)) % 180 == 90           // 角度差为0
					 && ((abs(lights[i].angle) == 0 && abs(lights[j].angle) == 90) || (abs(lights[i].angle) == 90 && abs(lights[j].angle) == 0))  // 角度1个为0 1个为90
					 && height_diff < 0.5 && width_diff < 0.5)               	  // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_			// 经验参数
					&& armor_ratio < 4                      // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20 )             // 经验参数
					{
						Armor_Twist armor_twist = MID_MOVE;
						LINE("[中等速度] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}// 1个竖着 1个横着
// 中速平移的情况 Mid Move

// 慢速移动的情况 Low Move
		// 都是竖着的
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.5
					 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.5
					 && static_cast<int>(abs(angle_diff)) % 180 == 0               // 角度差为0
					 && abs(lights[i].angle) == 0 && abs(lights[j].angle) == 0     // 角度为0 或 180
					 && height_diff < 0.5 && width_diff < 0.5)                  	 // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_	 // 经验参数
					&& armor_ratio < 4                   // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20 )          // 经验参数
					{
						Armor_Twist armor_twist = LOW_MOVE;
						LINE("[缓慢移动] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
		// 都是竖着的

		// 其中一块略有倾斜
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.5
					 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.5
					 &&	((abs(lights[i].angle) == 0  && abs(lights[j].angle) < 10) || (abs(lights[i].angle) < 10  && abs(lights[j].angle) == 0)) // 角度为0 或 180
					 && height_diff < 0.5 && width_diff < 0.5)                  	 // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_			// 经验参数
					&& armor_ratio < 4                      // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20               // 经验参数
					&& armor_light_angle_diff < 20 )
					{
						Armor_Twist armor_twist = LOW_MOVE;
						LINE("[缓慢移动] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
		// 其中一块略有倾斜

// 慢速移动的情况 Low Move

// 平凡的情况
	// 灯条近乎平行,至少在同一侧
			else if (lights[i].angle * lights[j].angle >= 0          // 灯条近乎同侧 , 或者有一个为0
				     && abs(angle_diff) < 30                           //  light_max_angle_diff_   // 20   // 18   这些都要换成相对值
					// && height_diff < _para.light_max_height_diff      // 20  不需要宽度
					)
			{
				cv::RotatedRect possible_rect;
				// 2灯条近乎平行 中速移动
				if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 1.5
					&& 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 1.5
					&& abs(light_aspect_ratio1 - light_aspect_ratio2) < 0.5 )
				{
					// 2灯条近乎平行 中速移动
					if (abs(lights[i].angle) > 60 && abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1                                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_                 // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = MID_MOVE;
							LINE("armor同侧,这是中速移动的armor,1-1.5,平躺")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 中速移动

					// 2灯条近乎平行 慢速移动
					else if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1                                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_                   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[缓慢移动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 中速移动

				// 2灯条近乎平行 快速移动
				else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 2.5
						 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 2.5
						 && abs(light_aspect_ratio1 - light_aspect_ratio2) < 1)
				{
					// 2灯条近乎平行 快速移动
					if (abs(lights[i].angle) > 60 && abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						
						if (armor_area > armor_min_area_
							&& armor_ratio > 1                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
						){
							Armor_Twist armor_twist = FAST_MOVE;
							LINE("[快速移动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					} // 2灯条近乎平行 快速移动

					// 2灯条近乎平行 慢速移动
					else if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1                                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_                   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[缓慢移动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 快速移动

				else if (light_min_aspect_ratio_ < light_aspect_ratio1    // && light_aspect_ratio1 < _para.light_max_aspect_ratio
						 && light_min_aspect_ratio_ < light_aspect_ratio2 // && light_aspect_ratio2 < _para.light_max_aspect_ratio
						 && (lights[i].center.y + lights[i].size.height / 2) > (lights[j].center.y - lights[j].size.height / 2)
						 && (lights[j].center.y + lights[j].size.height / 2) > (lights[i].center.y - lights[i].size.height / 2)
						 && abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
				{
					if (lights[i].center.x < lights[j].center.x)
						possible_rect = cv_toolbox_->boundingRRect(lights[i], lights[j]);
					else
				    possible_rect = cv_toolbox_->boundingRRect(lights[j], lights[i]);

					auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
					auto armor_angle = possible_rect.angle;
					auto armor_area = possible_rect.size.area();
					auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

					if (armor_area > armor_min_area_
						&& armor_ratio > 1                                         // _para.small_armor_min_ratio   // 1.5
				   		&& armor_ratio < 4.5                                       // armor_max_aspect_ratio_   // 3.0
				   		&& abs(armor_angle) < armor_max_angle_
				   		&& armor_light_angle_diff < armor_light_angle_diff_ ) // 应该要更为严格
					{
						LINE_INFO("angle_1",lights[i].angle)
						LINE_INFO("angle_2",lights[j].angle)
						LINE_INFO("angle_diff",angle_diff)
						LINE_INFO("height_diff", height_diff)
						LINE_INFO("width_diff",width_diff)
						LINE_INFO("armor_ratio",armor_ratio)
						LINE_INFO("armor_angle",armor_angle)
						LINE_INFO("armor_light_angle_diff",armor_light_angle_diff)
						
						Armor_Twist armor_twist = STILL;
						LINE("[几乎静止] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					}
				}
			} // 灯条严格平行
			
// 灯条(误差) 并不同侧
			else if (abs(angle_diff) < light_max_angle_diff_ )     // 40
			{
				cv::RotatedRect possible_rect;
				// 2灯条 中速移动
				if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 1.5
					&& 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 1.5
					&& abs(light_aspect_ratio1 - light_aspect_ratio2) < 0.5 )
				{
					// 2灯条不平行 慢速移动
					if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1  // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						)
						{
							Armor_Twist armor_twist = MID_MOVE;
							LINE("armor不同侧, 中速的armor, 竖直")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条不平行 慢速移动
				}// 2灯条 中速移动

				// 2灯条近乎平行 快速移动
				else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 2.5
						 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 2.5
						 && abs(light_aspect_ratio1 - light_aspect_ratio2) < 1)
				{
					if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ )// 应该要更为严格
						{
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[缓慢移动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 快速移动

				else if (light_min_aspect_ratio_ < light_aspect_ratio1 //&& light_aspect_ratio1 < _para.light_max_aspect_ratio
						 && light_min_aspect_ratio_ < light_aspect_ratio2 //&& light_aspect_ratio2 < _para.light_max_aspect_ratio
						 && (lights[i].center.y + lights[i].size.height / 2) > (lights[j].center.y - lights[j].size.height / 2)
						 && (lights[j].center.y + lights[j].size.height / 2) > (lights[i].center.y - lights[i].size.height / 2))
				{
					if (lights[i].center.x < lights[j].center.x)
						possible_rect = cv_toolbox_->boundingRRect(lights[i], lights[j]);
					else
				    possible_rect = cv_toolbox_->boundingRRect(lights[j], lights[i]);

					auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
					auto armor_angle = possible_rect.angle;
					auto armor_area = possible_rect.size.area();
					auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

					if (armor_area > armor_min_area_
						&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   		&& armor_ratio < 4.5 // armor_max_aspect_ratio_   // 3.0
				   		&& abs(armor_angle) < armor_max_angle_
				   		&& armor_light_angle_diff < armor_light_angle_diff_) // 应该要更为严格
					{
						LINE_INFO("angle_1",lights[i].angle)
						LINE_INFO("angle_2",lights[j].angle)
						LINE_INFO("angle_diff",angle_diff)
						LINE_INFO("height_diff", height_diff)
						LINE_INFO("width_diff",width_diff)
						LINE_INFO("armor_ratio",armor_ratio)
						LINE_INFO("armor_angle",armor_angle)
						LINE_INFO("armor_light_angle_diff",armor_light_angle_diff)
						Armor_Twist armor_twist = STILL;
						LINE("[几乎静止] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					}
				}
			} // 灯条(误差) 并不同侧
			else {
				cv::RotatedRect possible_rect;
				// 2灯条不平行 中速移动
				if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.5
					&& 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.5
					&& abs(light_aspect_ratio1 - light_aspect_ratio2) < 0.5 )
				{
					// 2灯条不平行 中速移动
					if (abs(lights[i].angle) > 60 &&  + abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						//auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			// && armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = MID_MOVE;
							LINE("[中速运动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条不平行 中速移动
				}
				else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 2.5
						 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 2.5
						 && abs(light_aspect_ratio1 - light_aspect_ratio2) < 1)
				{
					if (abs(lights[i].angle) > 60 && abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[慢速运动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 快速移动
			}
				
		} // for j loop
	} // for i loop

  if (enable_debug_)
  {
    for (int i = 0; i != armor_vector.size(); ++i)
      cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, armor_vector[i].rect, cv::Scalar(0,255,0), 2);
    cv::imshow("armors_before_filter", show_armors_befor_filter_);
  }
}
 void ConstraintSet::gammaProcessImage(cv::Mat& oriMat,double gamma,cv::Mat &outputMat){
   
    //伽马方法也是按照一个公式修改了每个像素值，我们可以通过LUT函数进行编写，它的公式是：
    //O=(I/255)的γ次方×255
    //代码如下
    cv::Mat lookupTable(1,256,CV_8U);
    uchar* p = lookupTable.ptr();
    for (int i =0 ; i < 256; i++) {
        p[i] = cv::saturate_cast<uchar>(pow(i/255.0, gamma) * 255.0);
    }
    LUT(oriMat,lookupTable,outputMat);
}
 int ConstraintSet::armor_score_best(std::vector<ArmorInfo> &armors)
   {
	std::vector<double> armor_score(armors.size());
	std::vector<double> armor_ratio_vec(armors.size());
	double width_average;
	for (int i = 0; i < armors.size(); i++){
		armor_ratio_vec[i] = std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height) / 
							  std::min(armors.at(i).rect.size.width, armors.at(i).rect.size.height);
         double size_check;
	    size_check=
	    -4.203*std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height)*
	    std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height)+
	    0.0383*std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height)+
	    1.2861;
        float sizescore;
	     if(fabs(armor_ratio_vec[i]-size_check)!=0)
	    {   
             if(armor_ratio_vec[i]<size_check){
				  if(pow(100/fabs(armor_ratio_vec[i]-size_check),2)>=21000)
				  sizescore=21000;
				  else sizescore=pow(100/fabs(armor_ratio_vec[i]-size_check),2);
				  armor_score[i]+=sizescore;
				                       }
	        else
	         {
                  if(pow(30/fabs(armor_ratio_vec[i]-size_check),2)>=21000)
				  sizescore=21000;
				  else sizescore=pow(30/fabs(armor_ratio_vec[i]-size_check),2);
				  armor_score[i]+=sizescore;
		     }
		}     
		else if(armor_ratio_vec[i]!=0)
		         {armor_score[i]+=21000;}         
		          if(fabs(armors.at(i).rect.angle)!=0){
				  float scoreangle=130/fabs(armors.at(i).rect.angle);
				 
				  if (10*pow(130/fabs(armors.at(i).rect.angle),2)>=26000)
				   scoreangle=26000;
				  else 
                    scoreangle=10*pow(130/fabs(armors.at(i).rect.angle),2);
					armor_score[i]+=scoreangle;}
				else 
				armor_score[i]+=26000;
		 double size_diff_score;
		       if(fabs(his_size-armor_ratio_vec[i])!=0&&his_size!=0)
		   {          if(1000/fabs(his_size-armor_ratio_vec[i])<20000)
					   size_diff_score=1000/fabs(his_size-armor_ratio_vec[i]);
				       else 
					   size_diff_score=20000;
					   armor_score[i]+=size_diff_score;

		   }	else  if (his_size!=0&&armor_ratio_vec[i]!=0)
		         armor_score[i]+=20000;
         double dis_score;
		      if(POINT_DIST(history_point,armors.at(i).rect.center)!=0)
		    {if(150000/POINT_DIST(history_point,armors.at(i).rect.center)>=40000)
				       dis_score=40000;
					else
					dis_score=150000/POINT_DIST(history_point,armors.at(i).rect.center);
					
					
					   if(history_point.x!=0){armor_score[i]+=dis_score;}
			}
			else if(history_point.x!=0){armor_score[i]+=40000;}
		width_average+=std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height);		 
		                                    }
         width_average/=armors.size(); 
		 for (int i = 0; i < armors.size(); i++) {

	     float area_score;
	     if (width_average>=32){area_score=-5*armors.at(i).rect.size.area();}
	     else 
	     area_score=-20*armors.at(i).rect.size.area();
         armor_score[i]+=area_score;}
        std::vector<double> dis_score(armors.size(), 0);
	for (int i = 0; i < armors.size(); i++) {

	  	for (int j = i + 1; j < armors.size(); j++) //  && (is_armor[j] == true)
		  { 
			   
			  double dis = fabs(POINT_DIST(armors.at(i).rect.center,armors.at(j).rect.center));
		
		    if(dis<=1.6*(std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height)+ 
					   std::max(armors.at(j).rect.size.width, armors.at(j).rect.size.height))&&
					   dis>=1*(std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height)+ 
					   std::max(armors.at(j).rect.size.width, armors.at(j).rect.size.height)))
					   {
						   if(armor_ratio_vec[i]>3.2&&armor_ratio_vec[j]<2.5)
						   armor_score[i]-=50000;
						   else if(armor_ratio_vec[j]>3.2&&armor_ratio_vec[i]<2.5)
						   armor_score[j]-=50000;
					   }
			if(dis<1*(std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height)+ 
					   std::max(armors.at(j).rect.size.width, armors.at(j).rect.size.height))&&
					   dis>=0.5*(std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height)+ 
					   std::max(armors.at(j).rect.size.width, armors.at(j).rect.size.height)))
					   {
						   if(armor_ratio_vec[i]>3.2&&armor_ratio_vec[j]<2.5)
						   armor_score[i]-=100000;
						   else if(armor_ratio_vec[j]>3.2&&armor_ratio_vec[i]<2.5)
						   armor_score[j]-=100000;
					   }
			if(dis!=0){
            dis_score[i]+=fabs((150000/dis)/ armors.size());
			dis_score[j]+=fabs((150000/dis)/ armors.size());}	

		
			
					   
				   
					   }}
			for(int i=0;i<armors.size();i++)
	{
		if(dis_score[i]>20000){
		   dis_score[i]=20000;
			
		}
		armor_score[i]+=dis_score[i];
	}
		int idx=0;
		for (int i=0;i<armors.size();i++)
		{if(armor_score[idx]<armor_score[i])
		idx=i;}
		//std::vector<double>::iterator biggest = std::max_element(std::begin(armor_score), std::end(armor_score));
			//	int idx=std::distance(std::begin(armor_score), biggest);
				return idx;
   }
  void ConstraintSet::FilterArmors(std::vector<ArmorInfo> &armors) {
	std::vector<bool> is_armor(armors.size(), true);

	for (int i = 0; i < armors.size(); i++) {
	  for (int j = i + 1; j < armors.size(); j++) {
			int ojbk = cv_toolbox_->armorToarmorTest(armors[i].rect, armors[j].rect);
			if (ojbk == 1) is_armor[i] = false;
			else if(ojbk == 2) is_armor[j] = false;
		}
	}
   std::vector<ArmorInfo> filter_rects;

	for( int i = 0; i < is_armor.size();++i)
		if(is_armor[i]) filter_rects.push_back(armors[i]);
	
	armors = filter_rects;
	if(armors.size()){filter_rects.clear();
	                 int idx=armor_score_best(armors);
					 float svm_result=get_armor_roi(armors.at(idx).rect);
					 if(svm_result>=0.1){
	                  filter_rects.push_back(armors[idx]);
					   history_point=armors.at(idx).rect.center;
		               his_size=std::max(armors.at(idx).rect.size.width, armors.at(idx).rect.size.height) / 
							  std::min(armors.at(idx).rect.size.width, armors.at(idx).rect.size.height);}
					  armors=filter_rects;}
	

	
    for (int i = 0; i != armors.size(); ++i)
      cv_toolbox_->DrawRotatedRect(src_img_, armors[i].rect, cv::Scalar(0,255,0), 2);
    //cv::imshow("armors_after_filter", src_img_);
  
}
  
/* 
 void ConstraintSet::FilterArmors(std::vector<ArmorInfo> &armors) 
{  


   
	


   if(armors.size()>0){
	std::vector<double> armor_score(armors.size(), 0);
	std::vector<double> armor_ratio_vec(armors.size(),0);
    std::vector<bool> is_armor(armors.size(), false);
   
	for (int i = 0; i < armors.size(); i++){
		armor_ratio_vec[i] = std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height) / 
							  std::min(armors.at(i).rect.size.width, armors.at(i).rect.size.height);
        double size_check;
	    size_check=
	    -4.203*std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height)*
	    std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height)+
	    0.0383*std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height)+
	    1.2861;
        float sizescore;
	    if(armor_ratio_vec[i]<=size_check)
	    {
				  if(pow(100/abs(armor_ratio_vec[i]-size_check),2)>=21000)
				  sizescore=21000;
				  else sizescore=pow(100/abs(armor_ratio_vec[i]-size_check),2);
				  armor_score[i]+=0.01*sizescore;
				                       }
	    else
	    {
                  if(pow(30/abs(armor_ratio_vec[i]-size_check),2)>=21000)
				  sizescore=21000;
				  else sizescore=pow(30/abs(armor_ratio_vec[i]-size_check),2);
				  armor_score[i]+=0.01*sizescore;
	    }
				  float scoreangle;
				  if (10*pow(130/abs(armors.at(i).rect.angle),2)>=26000)
				   scoreangle=26000;
				  else 
                    scoreangle=10*pow(130/abs(armors.at(i).rect.angle),2);
					armor_score[i]+=0.01*scoreangle;
	   
		;}
	
				std::vector<double>::iterator biggest = std::max_element(std::begin(armor_score), std::end(armor_score));
				int idx=std::distance(std::begin(armor_score), biggest);
				is_armor[idx]=true;
				printf("\n选定为%d\n",idx);
				std::vector<ArmorInfo>filter_rects;
				for( int i = 0; i < is_armor.size();++i)
		        if(is_armor[i]) filter_rects.push_back(armors[i]);
	            filter_rects.push_back(armors[idx]);
	            armors = filter_rects;
				filter_rects.clear();
                
				//armors = filter_rects;
				for (int i = 0; i != armors.size(); ++i)
          cv_toolbox_->DrawRotatedRect(src_img_, armors[idx].rect, cv::Scalar(0,255,0), 2);
          cv::imshow("armors_after_filter", src_img_);
				   }
	
		
}*/

	

ConstraintSet::~ConstraintSet() {

}

} // namesapce CC
