#ifndef DETECTION_NODE_H
#define DETECTION_NODE_H

#include <forward_list>
#include <functional>
#include <iostream>
#include <tuple>

#include <opencv2/opencv.hpp>


#include <ros/ros.h>
#include <ros/package.h>

#include <vector>

#include "utils.h"

namespace HYY{ 

#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))


void draw_rotated_rect(const cv::Mat &image, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness)
{
    cv::Point2f vertex[4];

    rect.points(vertex);
    for (int i=0; i<4; i++)
        cv::line(image, vertex[i], vertex[(i+1)%4], color, thickness);
}


namespace hyy_detection{

bool lights_x_comp(cv::RotatedRect &a, cv::RotatedRect &b)
{
    if (a.center.x < b.center.x)
    {
        return true;
    }
    else
    {
        return false;
    }
}

class ArmorDetector
{
private:
    cv::Mat frame;
    cv::Mat gray_img_;
    
    int armorFliterCnt;
    

    // Parameters come form .prototxt file
    bool enable_debug_;
    bool using_hsv_;
    bool enemy_color_;

    //! Use for debug
    cv::Mat show_lights_before_filter_;
    cv::Mat show_lights_after_filter_;
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

    bool isLight(cv::RotatedRect &armor_light ,cv::Mat &src_img_);
    cv::RotatedRect Light2Armor(cv::RotatedRect &Llight, cv::RotatedRect &Rlight);
    void adjustRect(cv::RotatedRect &rect);
    // bool lights_x_comp();

	//lights jugdement
	float parallel(cv::RotatedRect light1, cv::RotatedRect light2);
	float shapeSimilarity(cv::RotatedRect light1, cv::RotatedRect light2);
	float squareRatio(cv::RotatedRect light1, cv::RotatedRect light2);
	float yDis(cv::RotatedRect light1, cv::RotatedRect light2);
    float armor_size(cv::RotatedRect light1, cv::RotatedRect light2);

	//SVM
	// svm_big = StatModel::load<SVM>("config/big_armor_model.yml");		// do not have this file
	// svm_small = StatModel::load<SVM>("config/armor_model.yml");

public:
    std::vector<cv::RotatedRect> lights;
    std::vector<cv::RotatedRect> Flights;
    std::vector<cv::RotatedRect> armors;
    std::vector<cv::RotatedRect> his_armor;

    ArmorDetector();
    void LoadParam();
    void ArmorFinder(cv::Mat &img);
    void ArmorFliter(std::vector<cv::RotatedRect> &armors);
    int NumberRecognition(cv::Mat &frame);
    
    ~ArmorDetector();
};

ArmorDetector::ArmorDetector()
{
    armorFliterCnt = 0; 
    LoadParam();
}

ArmorDetector::~ArmorDetector()
{
}

void ArmorDetector::LoadParam()
{
  std::string file_name = ros::package::getPath("roborts_vision") + "/module/detect_factory/armor/constraint_set.xml";
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  if(!fs.isOpened())
  {
      ROS_ERROR ("Cannot open armor param file, please check if the file is exist000");
      return;
  }
      

  // algorithm info
  fs["enable_debug"] >> enable_debug_;
  fs["enemy_color"] >> enemy_color_;
  fs["using_hsv"] >> using_hsv_;

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
}

void ArmorDetector::ArmorFinder(cv::Mat &img)
{
	lights.clear();
    Flights.clear();
	armors.clear();
	cv::Mat frame = img;
    //detect mightly lights
    cv::cvtColor(frame, gray_img_, CV_BGR2GRAY);

    cv::Mat subtract_color_img;
	cv::Mat binary_brightness_img;
    cv::Mat binary_color_img;
    cv::Mat binary_light_img;

    std::vector<cv::Mat> bgr_channel;
	cv::split(frame, bgr_channel);

    if (enemy_color_ == 0) // red
		  cv::subtract(bgr_channel[2], bgr_channel[0], subtract_color_img);
	else
		  cv::subtract(bgr_channel[0], bgr_channel[2], subtract_color_img);
		
    cv::threshold(gray_img_, binary_brightness_img, light_threshold_, 255, CV_THRESH_BINARY);
    
    float thresh;
    if (enemy_color_ == 0) // red
    {
        thresh = red_threshold_;
    }
    else
    {
        thresh = blue_threshold_;
    }
      

    cv::threshold(subtract_color_img, binary_color_img, thresh, 255, CV_THRESH_BINARY);
    binary_light_img = binary_color_img & binary_brightness_img;

    // if (enable_debug_)
    // {
    // cv::imshow("color_img", binary_color_img);
    // cv::imshow("light_img", binary_brightness_img);
    // cv::imshow("lights_img", binary_light_img);
    // }
    
    std::vector<std::vector<cv::Point> > contours_light;
	cv::findContours(binary_light_img, contours_light, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> > contours_brightness;
	cv::findContours(binary_brightness_img, contours_brightness, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    for (unsigned int i = 0; i < contours_light.size(); ++i) {
        for (unsigned int j = 0; j < contours_brightness.size(); ++j) {
      	    if (cv::pointPolygonTest(contours_brightness[j], contours_light[i][0], true) >= 0.0) 
            {
                cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[j]);
                lights.push_back(single_light);
                break;
            }
        } // for j loop
    } // for i loop

    // ROS_ERROR("lights_number: %d", lights.size());
    if(lights.size() > 50){
        for (uchar cnt = 0; cnt < 50; cnt++)
        {
            if (isLight(lights[cnt], img))
            {
                Flights.push_back(lights[cnt]);
            }
        }
    }
    else{

        for (uchar cnt = 0; cnt < lights.size(); cnt++)
        {
            if (isLight(lights[cnt], img))
            {
                Flights.push_back(lights[cnt]);
            }
        }
    }
    
    
    // ROS_ERROR("???");
    // mightly armor

    if (Flights.size() < 2) return;
	std::vector<int> score(Flights.size(), 0);
    //test2
    {
    typedef std::tuple<float, size_t, size_t> Result;  // score, index1, index2
    size_t iEnd = Flights.size() - 1, jEnd = Flights.size();
    std::vector<Result> results(iEnd * jEnd / 2);
    auto p = results.begin();
    
    for (size_t i = 0; i < iEnd; ++i) {
        for (size_t j = i + 1; j < jEnd; ++j) {
            auto &l1 = Flights[i], &l2 = Flights[j];
            float score = squareRatio(l1, l2) * 5.0f
                          + yDis(l1, l2) * 8.0f
                          + shapeSimilarity(l1, l2) * 3.0f  //3.0
                          + parallel(l1, l2) * 1.2f         //1.2
                          + armor_size(l1, l2) * 0.5f;
            *(p++) = std::make_tuple(score, i, j);
        }
    }
    float winnerScore;
    size_t index1, index2;
    std::tie(winnerScore, index1, index2) = *std::min_element(results.begin(), results.end(),
                                                              [](const Result &a, const Result &b) {
                                                                  return std::get<0>(a) < std::get<0>(b);
                                                              });
    // ROS_ERROR("winnerScore: %f", winnerScore);
    if(winnerScore > 100) return;
    armors.push_back(Light2Armor(Flights[index1], Flights[index2]));

        // ROS_ERROR("Flights_number: %d", Flights.size() );
        // ROS_ERROR("armors_number: %d", armor_detector.armors.size() );

    
    }
	
}

void ArmorDetector::ArmorFliter(std::vector<cv::RotatedRect> &armors )	// SVM to confirm armor and get the armor number 
{
    
        
        // auto angle = armors[0].angle;
        if (armors[0].angle < -90)
        {
            armors[0].angle += 180;
        }
        else if (armors[0].angle > 90)
        {
            armors[0].angle -= 180;
        }
        auto Armorrect = std::minmax(armors[0].size.width, armors[0].size.height);
  	    auto armor_aspect_ratio = Armorrect.second / Armorrect.first;
        

        // ROS_ERROR("%f", armors[0].angle);
        // ROS_ERROR("area: %f", armors[0].size.area());
        
        if (!(1 < armor_aspect_ratio <= 2.5) )
        {
            armors.clear();
            return;
        }
        else if (!(std::abs(armors[0].angle) < 40) )    //40
        {
            armors.clear();
            return;
        }
        else if (!(armors[0].size.area() > 320 && armors[0].size.area() < 223000) )
        {
            armors.clear();
            return;
        }
        // else
        // {
            
        // }
        
                
    
}

int ArmorDetector::NumberRecognition(cv::Mat &frame)	/* SVM to detect number of armor */
{
    
}

bool ArmorDetector::isLight(cv::RotatedRect &armor_light ,cv::Mat &src_img_)
{
    
    adjustRect(armor_light);

    auto rect = std::minmax(armor_light.size.width, armor_light.size.height);
  	auto light_aspect_ratio = rect.second / rect.first;
    auto angle = armor_light.angle;

		if ( // 80 <= abs(angle) && abs(angle) <= 90   // 高速水平移动的灯条,带有拖影  // 特殊情况,无论横竖, 旧版本有这一行代码
		      light_aspect_ratio <= 2.5
		   && armor_light.size.area() > light_min_area_ // 1.0
		   && armor_light.size.area() < light_max_area_ * src_img_.size().height * src_img_.size().width) // 0.04
		{
			// ROS_ERROR("test1");
            return true;
		}
     // 针对灯条细小的情况, 没有最大比例的判断, 较为理想的灯条
		else if(armor_light.size.area() >= light_min_area_ // 1.0
		   		&& armor_light.size.area() < 100000  //light_max_area_ * src_img_.size().height * src_img_.size().width // 0.04
		   		&& abs(angle) < light_max_angle_) // 与垂直的偏角17.5 , 这里是可以取消/2的,进一步细化
		{
			// ROS_ERROR("test2");
            return true;
	    }
      // 检测最为平凡的情况
        else if (//light_aspect_ratio < _para.light_max_aspect_ratio  // 6.8
                    armor_light.size.area() >= light_min_area_ // 1.0
			     && armor_light.size.area() < light_max_area_ * src_img_.size().height * src_img_.size().width // 0.04
			     && abs(angle) < light_max_angle_) // 与垂直的偏角35 
        {
			// ROS_ERROR("test3");
            return true;
		}
        else
        {
            return false;
        }
        
    
}

void ArmorDetector::adjustRect(cv:: RotatedRect &rect)
{
    if(rect.size.width > rect.size.height)
    {
      auto temp = rect.size.height;
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

cv::RotatedRect ArmorDetector::Light2Armor(cv::RotatedRect &left, cv::RotatedRect &right)
{
    const cv::Point & pl = left.center, & pr = right.center;
	cv::Point2f center; 
	center.x = (pl.x + pr.x) / 2.0;
	center.y = (pl.y + pr.y) / 2.0;
	cv::Size2f wh_l = left.size;
	cv::Size2f wh_r = right.size;
	float width = POINT_DIST(pl, pr) - (wh_l.width + wh_r.width) / 2.0;
	float height = std::max(wh_l.height, wh_r.height);
	float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
	return cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / CV_PI);
}


float ArmorDetector::parallel(cv::RotatedRect light1, cv::RotatedRect light2) {
    return std::fabs(light1.angle - light2.angle) / 15.0f;
};

float ArmorDetector::shapeSimilarity(cv::RotatedRect light1, cv::RotatedRect light2) {
    float w1 = light1.size.width, h1 = light1.size.height;
    float w2 = light2.size.width, h2 = light2.size.height;
    float minWidth = std::min(w1, w2), minHeight = std::min(h1, h2);
    return std::fabs(w1 - w2) / minWidth + 0.33333f * std::fabs(h1 - h2) / minHeight;  // FIXME in python
};

float ArmorDetector::squareRatio(cv::RotatedRect light1, cv::RotatedRect light2) {
    float x1 = light1.center.x, y1 = light1.center.y;
    float x2 = light2.center.x, y2 = light2.center.y;
    float armorWidth = std::sqrt(utils::sqr(x1 - x2) + utils::sqr(y1 - y2));
    float armorHeight = 0.5f * (light1.size.height + light2.size.height);
    float ratio = armorWidth / armorHeight;
    if ( 1.0 > ratio || ratio >= 5.0)
    {
        return 1000;
    }
    // if ( ratio < 3.8 )
    // {
        return ratio > 0.85 ? utils::sqr(ratio - 2.5f) : 1e6f;
    // }
    // else
    // {
        // return ratio > 3.8 ? utils::sqr(ratio - 4.0f) : 1e6f;    //for big armor
    // }
    
};

float ArmorDetector::yDis(cv::RotatedRect light1, cv::RotatedRect light2) {
    float y1 = light1.center.y, y2 = light2.center.y;
    if (std::fabs((y1 - y2) > 3*std::min(light1.size.height, light2.size.height)))
    {
        return 1000;
    }
    
    // FIXME in python: y coordinates may be negetive
    return std::fabs((y1 - y2) / std::min(light1.size.height, light2.size.height));
};

float ArmorDetector::armor_size(cv::RotatedRect light1, cv::RotatedRect light2){
    float X = std::fabs(light1.center.x - light2.center.x);
    float H = std::fabs(std::max(light1.size.height, light2.size.height));
    // ROS_ERROR("armor_size: %f", 3600 / (X * H));
    return (111500 / (X * H));  //111500 gongye //3600 realsense
}

} //roborts_detection

} // namespace HYY

#endif