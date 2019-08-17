
#include<opencv2\opencv.hpp> 
 
using namespace std;
using namespace cv;
using namespace cv::ml;
 
int main()
{
	//训练需要用到的数据
	int labels[7] = { 1, 2, 3, 4, 5, 6, 7 };
	float train_data[7][2] = { { 31, 12 },{ 65, 220 },{ 440, 350 },{ 400, 400 } };

	//转为Mat以调用
	cv::Mat train_Mat(7, 64*64, CV_32FC1, train_data);
	cv::Mat label(7, 1, CV_32SC1, labels);


	//训练的初始化
	Ptr<SVM> svm = SVM::create();
	svm->setType(SVM::C_SVC);
	svm->setKernel(SVM::LINEAR);
	svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));
	//开始训练
	svm->train(训练Mat, ROW_SAMPLE, 标签label);
 


	//-----------无关紧要的美工的部分-----------------------	
	//----其实对每个像素点的坐标也进行了分类----------------
	int 宽 = 512, 高 = 512;
	Mat 演示图片 = Mat::zeros(高, 宽, CV_8UC3);
	Vec3b green(0, 255, 0), blue(255, 0, 0), red(0, 0, 255), black(0, 0, 0);
	for (int i = 0; i < 演示图片.rows; ++i)
		for (int j = 0; j < 演示图片.cols; ++j)
		{
			Mat sampleMat = (Mat_<float>(1, 2) << j, i);
			float response = svm->predict(sampleMat);
 
			if (response == 1)
				演示图片.at<Vec3b>(i, j) = green;
			else if (response == 2)
				演示图片.at<Vec3b>(i, j) = blue;
			else if (response == 3)
				演示图片.at<Vec3b>(i, j) = red;
			else if (response == 4)
				演示图片.at<Vec3b>(i, j) = black;
		}
	//--------把初始化训练的点画进图片------------
	int thickness = -1;
	int lineType = 8;
	for (int 画点 = 0; 画点 < sizeof(labels) / sizeof(int); 画点++) {
		circle(演示图片, Point(train_data[画点][0], train_data[画点][1]), 10, Scalar(255, 255, 255), thickness, -1);
	}
	// 把 support vectors  cout粗来看看……
	Mat sv = svm->getSupportVectors();
	cout << "Support Vectors为：" << endl;
	for (int i = 0; i < sv.rows; ++i)
	{
		const float* v = sv.ptr<float>(i);
		cout << v[0] << " " << v[1] << endl;
	}
 
	//测试测试
 
	Mat 结果;
	float teatData[2][2] = { { 20, 11 },{ 310, 411 } };
	Mat query(2, 2, CV_32FC1, teatData);
 
	svm->predict(query, 结果);
	cout << "分类结果为：" << endl;
	cout << 结果;
	imshow("SVM显示", 演示图片);
	waitKey(-1);
 
}
