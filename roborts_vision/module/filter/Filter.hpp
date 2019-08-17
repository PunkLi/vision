/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and 
to permit persons to whom the Software is furnished to do so, subject to the following conditions : 
The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

/**
 * Robomaster Vision program of RM2019
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#pragma once
#include <ros/ros.h>
#include <list>
#include <chrono>

class History
{
  const int8_t size_;
  int idx_;
  int8_t* raw_;
  int8_t* data_;
public:
  History(int8_t n = 5): size_(n), idx_(0) {
    raw_ = new int8_t[size_];
    data_ = new int8_t[size_];
    for(int8_t i = 0; i < size_; ++i) 
    {
      *(raw_ + i) = 0;
      *(data_ + i) = 0;
    }
  }
  void setLastResult(int8_t id) {
    *(raw_ + idx_) = id;
    idx_ = (idx_+1) % size_;
    memcpy(data_, raw_ + idx_, size_ - idx_);
    memcpy(data_ + size_ - idx_, raw_, idx_);
  }
  int8_t operator[](int8_t idx) {
    return *(data_ + size_ + idx - 1);
  }
};

template<typename TargetType>
class Filter
{
  const int history_size_;
  std::list<ros::Time> history_stamp; 
  std::list<TargetType> history_Target;
  int miss_detection_cnt;

public:
  Filter(int size_ = 5): history_size_(size_)
  {

  }
  void setLastResult(TargetType Target)
  {
    if (history_Target.size() < history_size_)
    {
      history_Target.push_back(Target);
      history_stamp.push_back(ros::Time());
    }
    else {
      history_Target.push_back(Target);           
      history_stamp.push_back(ros::Time());
      history_Target.pop_front();
      history_stamp.pop_front();
    }
  }
  void clear()
  {
    // history_Target.clear();
    // history_time.clear();
    if (history_Target.size() && history_stamp.size())
    {
      history_Target.pop_front();
      history_stamp.pop_front();
    }
  }
  int size()
  {
    return history_Target.size();
  }
  //virtual TargetType predict(); // predict new Target from history Target List
};