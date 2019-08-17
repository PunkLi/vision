#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Rune_fiall.h"

using namespace cv;
using namespace std;

//#define USE_VIDEO
//#define debug

//#define ENEMY_IS_RED      // 识别红色

float w0,L_R;
int color;
int colorwise;
int Maxarea ,Minarea ;
int Maxarea_l,Minarea_l;
int img_idx ;

RuneDetector::RuneDetector()
{
  std::string file_name = ros::package::getPath("roborts_vision") + "/module/detect_factory/rune/rune.xml";
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  if(!fs.isOpened())
    ROS_ERROR ("Cannot open rune param file, please check if the file is exist");
  else
    ROS_INFO ("open param file.....ok");

  //fs读取
  fs["enemy_color"] >> color;
  fs["Maxarea"] >> Maxarea;
  fs["Minarea"] >> Minarea;
  fs["Maxarea_l"] >> Maxarea_l;
  fs["Minarea_l"] >> Minarea_l;
  fs["img_idx"] >> img_idx;
}

void RuneDetector::Detect(uint8_t& vision_data_status,std::vector<RuneInfo>& armors)
{
    cv::cvtColor(src_img_, gray_image, COLOR_RGB2GRAY);

    contours_color_finall.clear();
    contours_color_l.clear();
    center_point.clear();
    contours_color.clear();

    vector<Mat> bgr_channel;
	cv::split(src_img_, bgr_channel);

#ifdef ENEMY_IS_RED
    cv::threshold(gray_image, binary_brightness_img, 80, 255, CV_THRESH_BINARY);
	cv::subtract(bgr_channel[2], bgr_channel[0], binary_color_img);
	cv::threshold(binary_color_img, binary_color_img, 60, 255, CV_THRESH_BINARY);
#else
    cv::threshold(gray_image, binary_brightness_img, 50, 255, CV_THRESH_BINARY);
	cv::subtract(bgr_channel[0], bgr_channel[2], binary_color_img);
    cv::threshold(binary_color_img,binary_color_img, 60, 255, CV_THRESH_BINARY);//130
#endif
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    cv::dilate(binary_brightness_img,binary_brightness_img,element);
    cv::GaussianBlur(binary_color_img,binary,Size(1,1),0,0);    
    binary = binary & binary_brightness_img;
    cv::dilate(binary,binary,element);

    findContours(binary, contours_color_l, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);//旋臂

#ifdef debug
    imshow("xuanbi",binary);
    waitKey(1);
#endif

    if(contours_color_l.empty())
    {
        //double time = ((double)getTickCount() - start) / getTickFrequency();
        //cout << time << "秒" << endl;
        cout<<"悬臂空"<<endl;
        return;
    }

    floodFill(binary,Point(3,3),Scalar(255));
    threshold(binary,binary_color_img,0,255,THRESH_BINARY_INV);//漫水算法

    morphologyEx(binary_color_img, binary_color_img, MORPH_OPEN, element);
    findContours(binary_color_img,contours_color, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

#ifdef debug
    imshow("manshui",binary_color_img);
    waitKey(1);
#endif

    for(int i=0;i<contours_color_l.size();i++)//旋臂筛选
    {
        double Cont = fabs(contourArea(contours_color_l[i],true));
         //if(Cont>300)
         //    cout<<"悬臂"<<Cont<<endl;
        if(Cont < Maxarea_l && Cont > Minarea_l){
            contours_color_finall.push_back(contours_color_l[i]);
            break;
        }
    }
    if(contours_color_finall.empty()){
        //double time = ((double)getTickCount() - start) / getTickFrequency();
        //cout << time << "秒" << endl;
        ROS_INFO("xuanbi.empty");
        return;
     }

    vector<vector<Point> >contours_ploy(contours_color.size());
    vector<RotatedRect> RotatedRect_ploy;
    RotatedRect predict_rect;

    float min,max;

    for(int i=0;i<contours_color.size();i++)
    {
        double Cont = fabs(contourArea(contours_color[i],true));
        // if(Cont>300)
        //     cout<<"漫水:"<<Cont<<endl;
        approxPolyDP(contours_color[i], contours_ploy[i], 5, true);//最小包围矩形
        RotatedRect temp1 = minAreaRect(contours_ploy[i]);
        if(temp1.size.width > temp1.size.height){
            min = temp1.size.height;
            max = temp1.size.width;
        }
        else{
            max = temp1.size.height;
            min = temp1.size.width;
        }

        float size;
        size = max/min;
        //cout<<size<<endl;
        if(size >1 && size< 4 && Cont < Maxarea && Cont > Minarea){//1.2 3
            RotatedRect_ploy.push_back(temp1);
        }
    }
    if(RotatedRect_ploy.empty())
    {
        cout<<"manshui.empty"<<endl;
        //double time = ((double)getTickCount() - start) / getTickFrequency();
       // cout << time << "秒" << endl;
        return ;
    }

#ifdef debug
    else{
        cout<<RotatedRect_ploy.size()<<endl;
    }
#endif
    Point2f pot[4],pots[4];
    float angle;
    for (int i = 0; i< RotatedRect_ploy.size(); i++)
    {
        RotatedRect_ploy[i].points(pot);

        if(pointPolygonTest(contours_color_finall[0],RotatedRect_ploy[i].center,false) == 1)//判断那个装甲板在旋臂里
        {
            predict_rect = RotatedRect_ploy[i];
            angle = RotatedRect_ploy[i].angle;
            // pot_cen = RotatedRect_ploy[i].center;
            for(int j=0; j<4; j++)
            {
                line(src_img_, pot[j], pot[(j+1)%4], Scalar(0,0,255),2);
            }
            break;
        }
    }

    //double time = ((double)getTickCount() - start) / getTickFrequency();
    //cout << time << "秒" << endl;

    contours_color.clear();
    contours_color_l.clear();
    contours_color_finall.clear();
    
    RuneInfo armor(predict_rect);
    armors.push_back(armor);

    if(!armors.empty()) {
      vision_data_status = 1;
    } 
    else{
        vision_data_status = 0;
    }
}

