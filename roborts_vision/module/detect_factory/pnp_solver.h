
#pragma once

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/package.h>

using namespace cv;
using namespace std;

class PnpSolver {
    cv::Mat cam_matrix_;
    cv::Mat dist_coeffs_;
    
    cv::Mat t_camera_ptz_;
    cv::Mat r_camera_ptz_;
   
    double scale_z_;

    double min_distance;
    double max_distance;

    double target_width_;
    double target_height_;

    std::vector<cv::Point3f> object3d_;
    cv::Point3f offset_;

public:
    PnpSolver(float width, float height)
    {
        // init object3d
        target_width_  = width;
        target_height_ = height;
        
        double half_x = target_width_  / 2.0;
        double half_y = target_height_ / 2.0;

        object3d_.push_back(cv::Point3f(-half_x, -half_y, 0));
        object3d_.push_back(cv::Point3f( half_x, -half_y, 0));
        object3d_.push_back(cv::Point3f( half_x,  half_y, 0));
        object3d_.push_back(cv::Point3f(-half_x,  half_y, 0));

        // load param
        //std::string file_name = ros::package::getPath("roborts_vision") + "/camera/uvc/uvc.xml";
        std::string file_name = ros::package::getPath("roborts_vision") + "/module/camera/mercure/mercure_pnp.xml";
        cv::FileStorage fs_camera(file_name, cv::FileStorage::READ);
        if(!fs_camera.isOpened())
            ROS_ERROR("Cannot open camera matrix and param file, please check if the file is exist.");
    
        fs_camera["Camera_Matrix"]           >> cam_matrix_;
        fs_camera["Distortion_Coefficients"] >> dist_coeffs_;
        
        //std::cout << "Camera_Matrix Size: \n" << cam_matrix_<< std::endl;
        //std::cout << "Distortion_Coefficients Size: \n" << dist_coeffs_ << std::endl;

        file_name = ros::package::getPath("roborts_vision") + "/module/detect_factory/offset.xml";
        cv::FileStorage fs_offset(file_name, cv::FileStorage::READ);
        if(!fs_offset.isOpened())
            ROS_ERROR("Cannot open offset.xml file, please check if the file is exist.");
        fs_offset["offset_x"] >> offset_.x;
        fs_offset["offset_y"] >> offset_.y;
        fs_offset["offset_z"] >> offset_.z;
        fs_offset["z_scale"]       >> scale_z_;
        fs_offset["min_distance"]  >> min_distance;
        fs_offset["max_distance"]  >> max_distance;
    }

    bool GetXYZ(cv::RotatedRect & rect, 
                cv::Point3f & target_3d, 
                cv::Point2f offset = cv::Point2f())
    {
        if (rect.size.height < 1) return false;

        double wh_ratio = target_width_ / target_height_;
        RotatedRect adj_rect(rect.center, Size2f(rect.size.width, rect.size.width/wh_ratio), rect.angle);
        
        std::vector<cv::Point2f> target2d;
        getTarget2d(rect, target2d, offset);

        cv::Mat rvec;
        cv::Mat tvec;
        cv::solvePnP(object3d_, target2d, cam_matrix_, dist_coeffs_, rvec, tvec);
        // Rodrigues(rvec, rot);
        //double enemy_ang = rvec.at<double>(1);//Rotate around y axis
    
        //ROS_INFO("Enemy angle: %f", enemy_ang * 180. / M_PI);

        target_3d.x = tvec.at<double>(0) + offset_.x; // cm
        target_3d.y = tvec.at<double>(1) + offset_.y; // cm
        target_3d.z = tvec.at<double>(2) + offset_.z; // * scale_z_;

        // target_3d.z = target_3d.z * scale_z_;

        if (target_3d.z < min_distance || max_distance < target_3d.z) {
            //ROS_WARN ("pnpsolver, out of distance range: [%f, %f]", min_distance, max_distance);
            return false;
        }
        return true;
    }

    void getTarget2d(const cv::RotatedRect & rect, 
                     std::vector<cv::Point2f> & target2d_,
                     cv::Point2f& offset)
    {
        cv::Point2f vertices[4];
        rect.points(vertices);
        cv::Point2f lu, ld, ru, rd;
        std::sort(vertices, vertices + 4, [](const cv::Point2f & p1, const cv::Point2f & p2) { return p1.x < p2.x; });
        if (vertices[0].y < vertices[1].y) {
            lu = vertices[0];
            ld = vertices[1];
        }
        else{
            lu = vertices[1];
            ld = vertices[0];
        }
        if (vertices[2].y < vertices[3].y) {
            ru = vertices[2];
            rd = vertices[3];
        }
        else {
            ru = vertices[3];
            rd = vertices[2];
        }

        target2d_.clear();
        target2d_.push_back(lu + offset);
        target2d_.push_back(ru + offset);
        target2d_.push_back(rd + offset);
        target2d_.push_back(ld + offset);
    }
};
