
// Copy from Opencv example 

/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include "train.h"

int main( int argc, char** argv )
{
    String sample0_dir = "../data/sample0"; // 负样本
    String sample1_dir = "../data/sample1";
    String sample2_dir = "../data/sample2";
    String sample3_dir = "../data/sample3";
    String sample4_dir = "../data/sample4";
    String sample5_dir = "../data/sample5";
    String sample6_dir = "../data/sample6";
    String sample7_dir = "../data/sample7";
    String sample8_dir = "../data/sample8";
   

    String svm_file = "../armor_model.yml";

    bool test_detector = false;
    bool train_twice = false;
    bool flip_samples = false; // 镜像反转 增加一倍的训练量

  std::vector<cv::Mat> sample_list0,
                       sample_list1,
                       sample_list2,
                       sample_list3,
                       sample_list4,
                       sample_list5,
                       sample_list6,
                       sample_list7,
                       sample_list8,   
                       full_neg_lst,
                       gradient_lst;   // 正样本的HOG

  std::vector<int> labels; // 标签
  load_images(sample0_dir, sample_list0, true);
  load_images(sample1_dir, sample_list1, true);
  load_images(sample2_dir, sample_list2, true);
  load_images(sample3_dir, sample_list3, true);
  /*load_images(sample4_dir, sample_list4, true);
  load_images(sample5_dir, sample_list5, true);
  load_images(sample6_dir, sample_list6, true);
  load_images(sample7_dir, sample_list7, true);
  load_images(sample8_dir, sample_list8, true);
  */
  cv::Size pos_image_size = cv::Size(64,64);
  pos_image_size = pos_image_size / 8 * 8;

  computeHOGs(pos_image_size, sample_list0, gradient_lst, false );
  size_t sample0_count = gradient_lst.size();
  labels.assign(sample0_count, -1);

  computeHOGs(pos_image_size, sample_list1, gradient_lst, false );
  size_t sample1_count = gradient_lst.size() - sample0_count;
  labels.insert(labels.end(), sample1_count, +2);
  
  computeHOGs(pos_image_size, sample_list2, gradient_lst, false );
  size_t sample2_count = gradient_lst.size() - sample0_count - sample1_count;
  labels.insert(labels.end(), sample2_count, +2);

  computeHOGs(pos_image_size, sample_list3, gradient_lst, false );
  size_t sample3_count = gradient_lst.size() - sample0_count - sample1_count - sample2_count;
  labels.insert(labels.end(), sample3_count, +3);
/* 
  computeHOGs(pos_image_size, sample_list4, gradient_lst, false );
  size_t sample4_count = gradient_lst.size() - sample0_count - sample1_count - sample2_count - sample3_count;
  labels.insert(labels.end(), sample4_count, +4);

  computeHOGs(pos_image_size, sample_list5, gradient_lst, false );
  size_t sample5_count = gradient_lst.size() - sample0_count - sample1_count - sample2_count - sample3_count - sample4_count;
  labels.insert(labels.end(), sample5_count, +5);

  computeHOGs(pos_image_size, sample_list7, gradient_lst, false );
  size_t sample7_count = gradient_lst.size() - sample0_count - sample1_count - sample2_count - sample3_count - sample4_count - sample5_count;
  labels.insert(labels.end(), sample7_count, +7);

  computeHOGs(pos_image_size, sample_list8, gradient_lst, false );
  size_t sample8_count = gradient_lst.size() - sample0_count - sample1_count - sample2_count - sample3_count - sample4_count - sample5_count - sample7_count;
  labels.insert(labels.end(), sample8_count, +8);
  */
  Mat train_data;
  convert_to_ml( gradient_lst, train_data );  // 将数据转换成ml训练的格式

  std::cout << "Training SVM...";
  Ptr< SVM > svm = SVM::create();

  /* Default values to train SVM */
  svm->setCoef0( 0.0 );
  svm->setDegree( 3 );
  svm->setTermCriteria( TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 1000, 1e-3 ) );
  svm->setGamma( 0 );
  svm->setKernel( SVM::LINEAR );
  svm->setNu( 0.5 );
  svm->setP( 0.1 ); // for EPSILON_SVR, epsilon in loss function?
  svm->setC( 0.01 ); // From paper, soft classifier
  svm->setType( SVM::EPS_SVR ); // C_SVC; // EPSILON_SVR; // may be also NU_SVR; // do regression task
  svm->train( train_data, ROW_SAMPLE, labels );
  
  std::cout << "...[done]" << endl;
  svm->save(svm_file);

  return 0;
}