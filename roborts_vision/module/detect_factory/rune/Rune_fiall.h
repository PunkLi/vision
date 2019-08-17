#pragma once

#include <fstream>
#include <string>
#include <math.h>
#include <thread>
#include <chrono>
#include <ctype.h>
#include <opencv2/ml.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <vector>
#include <list>

using namespace cv;
using namespace std;
using namespace cv::ml;
/* 

*/
class RuneInfo{
 public:
  RuneInfo(cv::RotatedRect armor_rect){
    rect = armor_rect;
  }
 public:
  cv::RotatedRect rect;
};


class RuneDetector{
public: 
    vector<vector<Point> > contours_color;
    vector<Point2f> center_point;
    vector<vector<Point> > contours_color_l;
    vector<vector<Point> > contours_color_finall;

    Point2f his_cen_point;
    Point2f predict_point;
    Point2f his;

    //Ptr<SVM> svm_small = StatModel::load<SVM>("../rune_svm.yml");

    Mat src_img_;
    Mat binary_color_img;
    Mat gray_image;
    Mat binary_brightness_img;
    Mat binary;

public:

    RuneDetector();

    void Detect(uint8_t&,vector<RuneInfo>& armors);

    Point2f get_center_point(vector<Point> cen_r){
        Moments mu;  
        mu = moments(cen_r, false);	
        Point2f mc;
        mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00); 
        return mc;
    }

    float distance_hanshu(Point2f points,Point2f cen_R){
        float distances = sqrt((points.x-cen_R.x)*(points.x-cen_R.x) + (points.y-cen_R.y)*(points.y-cen_R.y));
        return distances;
    }
};
