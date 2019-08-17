  /**
 * Robomaster Vision program of RM2019
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include "VisionNode.h"
  
#define USE_PREDICT // 使用预测

#define NeigborHood 640000

void VisionNode::ArmorFilter(std::vector<Detect_Factory::ArmorInfo> armors)
{
  bool is_same = false;
  std::vector<Detect_Factory::ArmorInfo> armors_inhood;

  if(armors.size())  // ?????-1
  {
    ROS_WARN("line: %d: %d%d%d%d%d-detect. ", __LINE__, 
              history_[-4], history_[-3], history_[-2], history_[-1], history_[0]);

    if (history_[0]) // ????1-1
    {
      DrawNeighborhood(image_raw_, last_armor_, cv::Scalar(0,255,255), 2); // starting 2nd...

      // step1: find armor
      for(int i = 0; i < armors.size(); ++i)
      {
        TransformToChassis(armors[i], temp_stamp_);
        double distance = GetPoseDistance(temp_stamp_, last_armor_in_chassis_);
        ROS_INFO("line: %d: distance of neighborhood= %f cm", __LINE__, sqrt(distance));

        if (distance < NeigborHood) { // 邻域判断
          if(armors[i].id == last_armor_.id) { // 判定同一辆车
            is_same = true;
            current_armor_ = armors[i];
            TransformToChassis(current_armor_, current_armor_in_chassis_);
            break; // 已经选到i了，就选出循环吧
          } else { 
            ROS_INFO("line: %d: this armor in neighborhood. but last [id = %d] now [id = %d].", __LINE__, last_armor_.id, armors[i].id); 
            armors_inhood.push_back(armors[i]); // 先把邻域的全部放进一个新的vector，然后再二次选择
          }
        } else { // 不在邻域的装甲，就不管他了
          ROS_WARN("line: %d: this armor out neighborhood. [id = %d]", __LINE__, armors[i].id); 
        }
      }
      // step2: 分类讨论
      if(is_same)                      // step2-A 领域内找到和上一帧同数字的装甲片
      {
        cv::Point3f target_3d; // 要打击的目标
        ROS_INFO("line: %d: we found armor has same id = %d.", __LINE__, current_armor_.id);
       
        if (history_[-1])  // ???11-1
        {
          ROS_INFO("line: %d: start predict...", __LINE__);
            
          //current_armor_in_chassis_.pose.position.x += 0.1 *(current_armor_in_chassis_.pose.position.x - last_armor_in_chassis_.pose.position.x);
          //current_armor_in_chassis_.pose.position.y += 0.1 *(current_armor_in_chassis_.pose.position.y - last_armor_in_chassis_.pose.position.y);
            
          this->TransformToGimbal();
          double distance = GetPoseDistance(current_armor_in_chassis_, last_armor_in_chassis_);
          ROS_INFO("line: %d: predict diff distance = %f cm", __LINE__, sqrt(distance));
  
          if (distance < NeigborHood) {
            ROS_INFO("line: %d: predict is ok.", __LINE__);
            GetPointFormPoseStamp(target_3d, current_armor_in_gimbal_); // 新预测的点
          } else {
            ROS_ERROR("line: %d: predict out of range [500cm]", __LINE__);
            target_3d = current_armor_.pt; // 如果距离相差太大 认为识别跳动了，不是同一块，那就不预测 x
          }
        }
        else {             // ???01-1
          ROS_INFO("line: %d: not predict...", __LINE__);
          target_3d = current_armor_.pt; 
        }
        gimbal_ctrl_.mode = 1; //UpateShootingState(target_3d);
        gimbal_contrl.SolveContrlAgnle(target_3d, gimbal_ctrl_.yaw, gimbal_ctrl_.pitch);
        gimbal_ctrl_.yaw = gimbal_ctrl_.yaw * 0.5;
        gimbal_ctrl_.pitch = gimbal_ctrl_.pitch * 0.3;
      }
      else if (armors_inhood.size())   // step2-B 邻域内找到的装甲片和上一帧不同，可能是svm出错了，需要纠正
      {
        ROS_WARN("line: %d: no same id armor in neighborhood.", __LINE__);
        int min_id = SelectNearestArmor(armors_inhood, last_armor_in_chassis_);
        ROS_INFO("line: %d: We choose nearlest armor.", __LINE__);
        current_armor_ = armors_inhood[min_id];
        TransformToChassis(current_armor_, current_armor_in_chassis_);
        //target_3d = current_armor_.pt; // 不预测
          
        // step 纠错
        if (history_[-1]) { // ???11-1
          ROS_WARN("line: %d: we think svm do mistake this frame. We change id = %d to id = %d", __LINE__, current_armor_.id, last_armor_.id);
          current_armor_.id = last_armor_.id;
        } else {             // ???01-1
          ROS_INFO("Maybe last svm take mistake. still setLastResult. "); // 该分支是连续识别的 2rd
        }
        gimbal_ctrl_.mode = 1; //UpateShootingState(target_3d);
        gimbal_contrl.SolveContrlAgnle(current_armor_.pt, gimbal_ctrl_.yaw, gimbal_ctrl_.pitch);
        gimbal_ctrl_.yaw = gimbal_ctrl_.yaw * 0.5;
        gimbal_ctrl_.pitch = gimbal_ctrl_.pitch * 0.3;
      }
      else{ //                         // step2-C 邻域内没有
        ROS_WARN("line: %d: step2-C", __LINE__);
        gimbal_ctrl_.mode  = 0; //UpateShootingState(target_3d);
        gimbal_ctrl_.yaw   = 0;
        gimbal_ctrl_.pitch = 0;
      }    
    }
    else             // ????0-1
    {
      if (history_[-1] && history_[-2])       // ??110-1, 丢1帧, 仍当做3rd
      {
        // step1: find armor 
        for(int i = 0; i < armors.size(); ++i)
        {
          TransformToChassis(armors[i], temp_stamp_);
          double distance = GetPoseDistance(temp_stamp_, last_armor_in_chassis_);
          ROS_INFO("line: %d: distance of neighborhood= %f cm", __LINE__, sqrt(distance));

          if (distance < NeigborHood) { // 5m
            if(armors[i].id == last_armor_.id) { // 判定同一辆车
              is_same = true;
              current_armor_ = armors[i];
              TransformToChassis(current_armor_, current_armor_in_chassis_);
              break; // 已经选到i了，就选出循环吧
            } else { 
              ROS_INFO("line: %d: this armor in neighborhood. but last [id = %d] now [id = %d].", __LINE__, last_armor_.id, armors[i].id); 
              armors_inhood.push_back(armors[i]); // // 先把邻域的全部放进一个新的vector，然后再二次选择
            }
          } else { // 不在邻域的装甲，就不管他了
            ROS_WARN("line: %d: this armor out neighborhood. [id = %d]", __LINE__, armors[i].id);
          }
        }
        // step2: 分类讨论
        if(is_same)                      // step2-A 领域内找到和上一帧同数字的装甲片
        {
          cv::Point3f target_3d; // 要打击的目标
          ROS_INFO("line: %d: we found armor has same id = %d.", __LINE__, current_armor_.id);
        
          if (history_[-1])  // ???11-1
          {
            ROS_INFO("line: %d: start predict...", __LINE__);
              
            //current_armor_in_chassis_.pose.position.x += 0.1 *(current_armor_in_chassis_.pose.position.x - last_armor_in_chassis_.pose.position.x);
            //current_armor_in_chassis_.pose.position.y += 0.1 *(current_armor_in_chassis_.pose.position.y - last_armor_in_chassis_.pose.position.y);
              
            this->TransformToGimbal();
            double distance = GetPoseDistance(current_armor_in_chassis_, last_armor_in_chassis_);
            ROS_INFO("line: %d: predict diff distance = %f cm", __LINE__, sqrt(distance));
      
            if (distance < NeigborHood) {
              ROS_INFO("line: %d: predict is ok.", __LINE__);
              GetPointFormPoseStamp(target_3d, current_armor_in_gimbal_); // 新预测的点
            } else {
              ROS_ERROR("line: %d: predict out of range [500cm]", __LINE__);
              target_3d = current_armor_.pt; // 如果距离相差太大 认为识别跳动了，不是同一块，那就不预测 x
            }
          }
          else {             // ???01-1
            ROS_INFO("line: %d: not predict...", __LINE__);
            target_3d = current_armor_.pt; 
          }
          gimbal_ctrl_.mode = 1; //UpateShootingState(target_3d);
          gimbal_contrl.SolveContrlAgnle(target_3d, gimbal_ctrl_.yaw, gimbal_ctrl_.pitch);
          gimbal_ctrl_.yaw = gimbal_ctrl_.yaw * 0.5;
          gimbal_ctrl_.pitch = gimbal_ctrl_.pitch * 0.3;
        }
        else if (armors_inhood.size())   // step2-B 邻域内找到的装甲片和上一帧不同，可能是svm出错了，需要纠正
        {
          ROS_WARN("line: %d: no same id armor in neighborhood.", __LINE__);
          int min_id = SelectNearestArmor(armors_inhood, last_armor_in_chassis_);
          ROS_INFO("line: %d: We choose nearlest armor.", __LINE__);
          current_armor_ = armors_inhood[min_id];
          TransformToChassis(current_armor_, current_armor_in_chassis_);
          //target_3d = current_armor_.pt; // 不预测
          
          // step 纠错
          if (history_[-1]) { // ???11-1
            ROS_WARN("line: %d: we think svm do mistake this frame. We change id = %d to id = %d", __LINE__, current_armor_.id, last_armor_.id);
            current_armor_.id = last_armor_.id;
          } else {             // ???01-1
            ROS_INFO("Maybe last svm take mistake. still setLastResult. "); // 该分支是连续识别的 2rd
          }
          gimbal_ctrl_.mode = 1; //UpateShootingState(target_3d);
          gimbal_contrl.SolveContrlAgnle(current_armor_.pt, gimbal_ctrl_.yaw, gimbal_ctrl_.pitch);
          gimbal_ctrl_.yaw = gimbal_ctrl_.yaw * 0.5;
          gimbal_ctrl_.pitch = gimbal_ctrl_.pitch * 0.3;
        }
        else{ //                         // step2-C 邻域内没有
          ROS_WARN("line: %d: step2-C", __LINE__);
          gimbal_ctrl_.mode  = 0; //UpateShootingState(target_3d);
          gimbal_ctrl_.yaw   = 0;
          gimbal_ctrl_.pitch = 0;
        }    
      }
      else if (history_[-2] && history_[-3])  // ?1100-1, 丢2帧, 仍继续识别
      {
        // step1: find armor 
        for(int i = 0; i < armors.size(); ++i)
        {
          TransformToChassis(armors[i], temp_stamp_);
          double distance = GetPoseDistance(temp_stamp_, last_armor_in_chassis_);
          ROS_INFO("line: %d: distance of neighborhood= %f cm", __LINE__, sqrt(distance));

          if (distance < NeigborHood) { // 5m
            if(armors[i].id == last_armor_.id) { // 判定同一辆车
              is_same = true;
              current_armor_ = armors[i];
              TransformToChassis(current_armor_, current_armor_in_chassis_);
              break; // 已经选到i了，就选出循环吧
            } else { 
              ROS_INFO("line: %d: this armor in neighborhood. but last [id = %d] now [id = %d].", __LINE__, last_armor_.id, armors[i].id); 
              armors_inhood.push_back(armors[i]); // // 先把邻域的全部放进一个新的vector，然后再二次选择
            }
          } else { // 不在邻域的装甲，就不管他了
            ROS_WARN("line: %d: this armor out neighborhood. [id = %d]", __LINE__, armors[i].id);
          }
        }

        if(is_same)                     // step2-A 领域内找到和上一帧同数字的装甲片
        {
          ROS_INFO("line: %d: we found armor has same id = %d.", __LINE__, current_armor_.id);
          ROS_INFO("line: %d: start predict...", __LINE__);
            
          //current_armor_in_chassis_.pose.position.x += 0.1 *(current_armor_in_chassis_.pose.position.x - last_armor_in_chassis_.pose.position.x);
          //current_armor_in_chassis_.pose.position.y += 0.1 *(current_armor_in_chassis_.pose.position.y - last_armor_in_chassis_.pose.position.y);
              
          this->TransformToGimbal();  
          double distance = GetPoseDistance(current_armor_in_chassis_, last_armor_in_chassis_);
          ROS_INFO("between predict with truth, distance = %f cm", sqrt(distance));
          
          cv::Point3f target_3d;
          if (distance < NeigborHood) { // 500cm
            ROS_WARN("predict is ok.");
            GetPointFormPoseStamp(target_3d, current_armor_in_gimbal_); // 新预测的点
          } else {
            ROS_ERROR("predict maybe out of range [500cm]");
            target_3d = current_armor_.pt; // 如果距离相差太大 认为识别跳动了，不是同一块，那就不预测 x
          }
          gimbal_ctrl_.mode = 1; //UpateShootingState(target_3d);
          gimbal_contrl.SolveContrlAgnle(target_3d, gimbal_ctrl_.yaw, gimbal_ctrl_.pitch);
          gimbal_ctrl_.yaw = gimbal_ctrl_.yaw * 0.5;
          gimbal_ctrl_.pitch = gimbal_ctrl_.pitch * 0.3;

        }
        else if (armors_inhood.size())  // step2-B 邻域内找到的装甲片和上一帧不同，置信度不高，不纠正
        {
          ROS_WARN("line: %d: armor in neighborhood. but not same id.", __LINE__);
          int min_id = SelectNearestArmor(armors_inhood, last_armor_in_chassis_);
          ROS_INFO("line: %d: We choose nearlest armor.", __LINE__);
          current_armor_ = armors_inhood[min_id];
          TransformToChassis(current_armor_, current_armor_in_chassis_);
          //target_3d = current_armor_.pt; // 不预测
          gimbal_ctrl_.mode = 1; //UpateShootingState(target_3d);
          gimbal_contrl.SolveContrlAgnle(current_armor_.pt, gimbal_ctrl_.yaw, gimbal_ctrl_.pitch);
          gimbal_ctrl_.yaw = gimbal_ctrl_.yaw * 0.5;
          gimbal_ctrl_.pitch = gimbal_ctrl_.pitch * 0.3;
        }
        else{                           // step2-C 邻域内没有
          ROS_WARN("line: %d: step2-C", __LINE__);
          gimbal_ctrl_.mode  = 0; //UpateShootingState(target_3d);
          gimbal_ctrl_.yaw   = 0;
          gimbal_ctrl_.pitch = 0;
        }
      }
      else {                                  // ?0100-1 or ?1000-1 // 我们认为这一帧是误识别 or 第一帧           
        ROS_INFO("line: %d: maybe we [first detect]", __LINE__);
        // step1: find armor
        if (armors.size() == 1)
          current_armor_ = armors[0]; // 此时，我们要重新选择第一帧的目标，全图搜索
        else if(armors.size() == 2)
          current_armor_ = ChooseArmorById(armors[0],armors[1]);
        else {
          // 先按照 x^2 + y^2 + z^2 排序
          std::sort(armors.begin(),armors.end(),
            [](Detect_Factory::ArmorInfo armor1, Detect_Factory::ArmorInfo armor2) {
              double dis1 = armor1.pt.x * armor1.pt.x + armor1.pt.y * armor1.pt.y + armor1.pt.z * armor1.pt.z;
              double dis2 = armor2.pt.x * armor2.pt.x + armor2.pt.y * armor2.pt.y + armor2.pt.z * armor2.pt.z;
              if(dis1 < dis2)
                return true;
              else
                return false;
            });
        current_armor_ = ChooseArmorById(armors[0],armors[1]);
        if (current_armor_.id == 2)
          current_armor_ = ChooseArmorById(armors[0],armors[2]);
        }
        ROS_INFO("line: %d: we found armor, id = %d", __LINE__, current_armor_.id);
        // step2: tf to chassis
        TransformToChassis(current_armor_, current_armor_in_chassis_);
        
        gimbal_ctrl_.mode  = 0; //UpateShootingState(target_3d);
        //gimbal_contrl.SolveContrlAgnle(current_armor_.pt, gimbal_ctrl_.yaw,gimbal_ctrl_.pitch);
        gimbal_ctrl_.yaw   = 0;
        gimbal_ctrl_.pitch = 0;
      }
    }  
    history_.setLastResult(current_armor_.id);          // 更新
    last_armor_            = current_armor_;            // 更新
    last_armor_in_chassis_ = current_armor_in_chassis_; // 更新
  }
  else{
    ROS_WARN("line: %d: %d%d%d%d%d-miss. ", __LINE__, 
            history_[-4], history_[-3], history_[-2], history_[-1], history_[0]);
      
    if (history_[0] && history_[-1])                            // ???11-0
    {
      gimbal_ctrl_.mode = TaskState::SHOOT_ONCE;
      ROS_INFO("line: %d: We have many history data. Maybe we lost 1 frame now, TaskState::SHOOT_ONCE", __LINE__); 
    }
    else if(history_[0] || history_[-1])                        // ???10-0   ???01-1 
    {
      gimbal_ctrl_.mode = TaskState::SHOOT_STOP;
      if(history_[-2] && history_[-3])                          // ?1101-0   ?1110-0
        ROS_INFO("line: %d: We lost 1 or 2.....", __LINE__);
      else if (history_[-2] || history_[-3])                    // ?0101-0   ?1001-0 
        ROS_INFO("line: %d: We unknown.....", __LINE__);
    }
    else{
      gimbal_ctrl_.mode = TaskState::AWAIT;
      gimbal_ctrl_.yaw = 0;
      gimbal_ctrl_.pitch = 0;
      ROS_ERROR("line: %d: We lost target.", __LINE__); 
    }
    history_.setLastResult(0);
  }
    ROS_WARN("gimbal_ctrl_.mode \t %d", gimbal_ctrl_.mode);
    ROS_INFO("gimbal_ctrl_.yaw \t %f",  gimbal_ctrl_.yaw);
    ROS_INFO("gimbal_ctrl.pitch \t %f", gimbal_ctrl_.pitch);
    ros_pub_gimbal.publish(gimbal_ctrl_);
  }
  
  void VisionNode::GetArmorPoseStamped(const Detect_Factory::ArmorInfo & armor, geometry_msgs::PoseStamped & stamp)
  {
    stamp.header.stamp       = ros::Time();
    stamp.header.frame_id    = gimbal_link_;
    stamp.pose.position.x    =  armor.pt.z;
    stamp.pose.position.y    = -armor.pt.x;
    stamp.pose.position.z    = -armor.pt.y;

    float yaw = stamp.pose.position.y / stamp.pose.position.x;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);

    stamp.pose.orientation.x = quaternion.x();
    stamp.pose.orientation.y = quaternion.y();
    stamp.pose.orientation.z = quaternion.z();
    stamp.pose.orientation.w = quaternion.w();
  }

void VisionNode::TransformToChassis(Detect_Factory::ArmorInfo & armor, 
                                    geometry_msgs::PoseStamped & target_stamp)
{
  geometry_msgs::PoseStamped stamp;
  GetArmorPoseStamped(armor, stamp);
  try{
    tf_.transformPose("base_link", stamp, target_stamp);
    //ROS_INFO("armor id : %d", armor.id);
    //ROS_INFO("armor in chassis -> x : %f cm", target_stamp.pose.position.x);
    //ROS_INFO("armor in chassis -> y : %f cm", target_stamp.pose.position.y);
    //ROS_INFO("armor in chassis -> z : %f cm", target_stamp.pose.position.z);
  }
  catch (tf::TransformException &ex){
    ROS_ERROR("%s",ex.what());
    ROS_ERROR("tf error when transform armor pose from /gimbal_link to /base_link");
    ros::Duration(1.0).sleep();
  }
}

void VisionNode::TransformToGimbal()
{
  try{
    tf_.transformPose(gimbal_link_, current_armor_in_chassis_, current_armor_in_gimbal_);  // 重新回到云台坐标系 v
  }
  catch (tf::TransformException &ex){
    ROS_ERROR("%s",ex.what());
    ROS_ERROR("tf error when transform enemy pose from /base_link to /gimbal_link");
    ros::Duration(1.0).sleep();
  }
}
  
int VisionNode::SelectNearestArmor(std::vector<Detect_Factory::ArmorInfo> & armors, geometry_msgs::PoseStamped & stamp)
{
  double min_dis = 1000000;
  int min_id = -1;
  for(int i = 0; i < armors.size(); ++i)
  {
    TransformToChassis(armors[i], temp_stamp_);
    double distance = GetPoseDistance(temp_stamp_, stamp);
    if(distance < min_dis)
    {
      min_dis = distance;
      min_id = i;
    }
  }
  return min_id;
}

  double VisionNode::GetPoseDistance(const geometry_msgs::PoseStamped & stamp1, 
                         const geometry_msgs::PoseStamped & stamp2) {
    return 
      (stamp1.pose.position.x - stamp2.pose.position.x) *
      (stamp1.pose.position.x - stamp2.pose.position.x) +
      (stamp1.pose.position.y - stamp2.pose.position.y) *
      (stamp1.pose.position.y - stamp2.pose.position.y);
  }

void VisionNode::GetPointFormPoseStamp(cv::Point3f& target_3d, 
                             const geometry_msgs::PoseStamped & stamp)
{
  target_3d.x = -stamp.pose.position.y;
  target_3d.y = -stamp.pose.position.z; 
  target_3d.z =  stamp.pose.position.x;
}

  void VisionNode::DrawNeighborhood(const cv::Mat &img, Detect_Factory::ArmorInfo &armor, const cv::Scalar &color, int thickness) {

    cv::Point2f center = armor.rect.center; 
    cv::Size2f size = armor.rect.size;
  
    float width = size.width * 4.0;
    float height = size.height * 4.0;

    cv::RotatedRect rect(center, cv::Size2f(width, height), 0 * 180 / CV_PI);

    cv::Point2f vertex[4];

    rect.points(vertex);
    for (int i = 0; i < 4; i++)
      cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
  }

void VisionNode::UpateShootingState(const cv::Point3f & target_3d)
{
  if(gimbal_link_ == "small_gimbal_link")
  {
    if (target_3d.z > 500)      
    {
      ROS_WARN("Before fric speed = %f, now We change to 25, target_z = %f", gimbal_info_.fric_speed, target_3d.z);
      gimbal_ctrl_.fric_speed = 25;  // 大于5米的提速到25
    }
    else if (target_3d.z < 150) 
    {
      ROS_WARN("Before fric speed = %f, now We change to 12, target_z = %f", gimbal_info_.fric_speed, target_3d.z);
      gimbal_ctrl_.fric_speed = 12;  // 小于2米的降速到15
    }
    else
      gimbal_ctrl_.fric_speed = gimbal_info_.fric_speed; // 保持当前的射速
  }
  double dis = target_3d.x * target_3d.x + target_3d.y * target_3d.y;
  if(dis < 100)
  {
    ROS_WARN("SHOOT_CONTINUOUS");
    gimbal_ctrl_.mode = TaskState::SHOOT_CONTINUOUS;
  }
  else if(dis < 300)
  {
    ROS_WARN("SHHOT_THREE");
    gimbal_ctrl_.mode = TaskState::SHHOT_THREE;
  }
  else if(dis < 500)
  {
    ROS_WARN("SHOOT_TWICE");
    gimbal_ctrl_.mode = TaskState::SHOOT_TWICE;
  }
  else if(dis < 900)
  {
    ROS_WARN("SHOOT_ONCE");
    gimbal_ctrl_.mode = TaskState::SHOOT_ONCE;
  }
  else{
    ROS_WARN("SHOOT_STOP");
    gimbal_ctrl_.mode = TaskState::SHOOT_STOP;
  }
}

Detect_Factory::ArmorInfo& VisionNode::ChooseArmorById(Detect_Factory::ArmorInfo& armor1, 
                                                       Detect_Factory::ArmorInfo& armor2)
{
  if(armor1.id == armor2.id) return armor1;

  if(armor1.id == 1) return armor1;
  if(armor2.id == 1) return armor2;

  if(armor1.id == 2) return armor2;
  if(armor2.id == 2) return armor1;
}