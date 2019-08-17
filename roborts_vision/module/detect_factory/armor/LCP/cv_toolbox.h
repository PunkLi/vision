/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify *  it under the terms of the GNU General Public License as published by *  the Free Software Foundation, either version 3 of the License, or *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#pragma once

#include <vector>
#include <thread>
#include <mutex>
//opencv
#include <opencv2/opencv.hpp>
//ros
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace LCP{

#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))

class CVToolbox {
 public:
  void DrawRotatedRect(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness) {
    cv::Point2f vertex[4];

    cv::Point2f center = rect.center;
    float angle = rect.angle;
    std::ostringstream ss;
    ss << angle;
    std::string text(ss.str());
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 0.5;
    cv::putText(img, text, center, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);

    rect.points(vertex);
    for (int i = 0; i < 4; i++)
      cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
  }

  void DrawRotatedRect(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness, float angle) {
    cv::Point2f vertex[4];

    cv::Point2f center = rect.center;
    std::ostringstream ss;
    ss << (int)(angle);
    std::string text(ss.str());
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 0.5;
    cv::putText(img, text, center, font_face, font_scale, cv::Scalar(0, 100, 0), thickness, 8, 0);

    rect.points(vertex);
    for (int i = 0; i < 4; i++)
      cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
  }

  

  void DrawTarget3d(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness, cv::Point3f pt) {
    cv::Point2f vertex[4];

    cv::Point2f center = rect.center;
    std::ostringstream ss;
    ss << pt.x << ", " << pt.y << ", " << pt.z;
    std::string text(ss.str());

    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 0.5;
    cv::putText(img, text, center, font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);

    rect.points(vertex);
    for (int i = 0; i < 4; i++)
      cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
  }

  void adjustRect(cv:: RotatedRect &rect)
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
  /**
   * @brief: 针对平凡的情况
   */
  cv::RotatedRect boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right){
    const cv::Point & pl = left.center, & pr = right.center;
    cv::Point2f center; 
    center.x = (pl.x + pr.x) / 2.0;
    center.y = (pl.y + pr.y) / 2.0;
    cv::Size2f wh_l = left.size;
    cv::Size2f wh_r = right.size;
    float width = POINT_DIST(pl, pr) - (wh_l.width + wh_r.width) / 2.0;
    float height = std::max(wh_l.height, wh_r.height) + std::max(wh_l.height, wh_r.height);
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / CV_PI);
  }
  /**
   * @brief: 针对快速平移的情况
   */
  cv::RotatedRect boundingRRectFast(const cv::RotatedRect & left, const cv::RotatedRect & right){
    const cv::Point & pl = left.center, & pr = right.center;
    cv::Point2f center; 
    center.x = (pl.x + pr.x) / 2.0;
    center.y = (pl.y + pr.y) / 2.0;
    cv::Size2f wh_l = left.size;
    cv::Size2f wh_r = right.size;
    float width = POINT_DIST(pl, pr);// - (wh_l.width + wh_r.width) / 2.0;
    float height = std::max(wh_l.width, wh_r.width);
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / CV_PI);
  }
  /**
   * @brief: 针对慢速平移的情况
   */
  cv::RotatedRect boundingRRectSlow(const cv::RotatedRect & left, const cv::RotatedRect & right){
    const cv::Point & pl = left.center, & pr = right.center;
    cv::Point2f center; 
    center.x = (pl.x + pr.x) / 2.0;
    center.y = (pl.y + pr.y) / 2.0;
    cv::Size2f wh_l = left.size;
    cv::Size2f wh_r = right.size;
    float width = POINT_DIST(pl, pr);// - (wh_l.width + wh_r.width) / 2.0;
    float height = std::max(wh_l.height, wh_r.height);
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / CV_PI);
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
};

} // namespace LCP
