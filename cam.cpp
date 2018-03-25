#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include <string>
#include <iostream>

#define CAM_RES_X 640
#define CAM_RES_Y 360

using namespace std;
using namespace cv;
int main() {
	Mat frame(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat img(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat gray(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	vector<Mat> frame_split(4);
	VideoCapture capture;
	capture.open(0, CAP_V4L2);
	
	namedWindow("img", CV_WINDOW_AUTOSIZE);
	if (!capture.isOpened()){
		cout <<"error with camera"<<endl;
	}
	else
	{
		capture.set(CAP_PROP_FRAME_WIDTH,640);
		capture.set(CAP_PROP_FRAME_HEIGHT,360);
		//capture.set(CAP_PROP_MODE,CAP_MODE_YUYV);
		capture.set(CAP_PROP_CONVERT_RGB,false);
		//cout<<"MODE"<<CAP_MODE_YUYV<<endl;
		cout<<"MODE"<<capture.get(CAP_PROP_MODE)<<endl;
		cout<<"RGB"<<capture.get(CAP_PROP_CONVERT_RGB)<<endl;
	}
	


	while (true) {
		//YUYV-->GRAY;
		capture >> frame;
		cv::split(frame, frame_split);
		gray = frame_split[0];
		//frame.copyTo(img);
		//cvtColor(img, gray, CV_RGB2GRAY);
		imshow("img", gray);
		//cout<<img.size()<<endl;
		waitKey(33);
	}
	capture.release();
	return 0;
}

/*
int main() {
	Mat frame(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat img(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat gray(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	VideoCapture capture;
	capture.open(0, CAP_V4L2);
	
	namedWindow("img", CV_WINDOW_AUTOSIZE);
	if (!capture.isOpened()){
		cout <<"error with camera"<<endl;
	}
	else
	{
		capture.set(CAP_PROP_FRAME_WIDTH,640);
		capture.set(CAP_PROP_FRAME_HEIGHT,360);
		capture.set(CAP_PROP_MODE,CAP_MODE_RGB);
		capture.set(CAP_PROP_CONVERT_RGB,true);
		cout<<"MODE"<<capture.get(CAP_PROP_MODE)<<endl;
		cout<<"RGB"<<capture.get(CAP_PROP_CONVERT_RGB)<<endl;
	}
	


	while (true) {
		//YUYV-->GRAY;
		capture >> frame;
	
		frame.copyTo(img);
		//cvtColor(img, gray, CV_RGB2GRAY);
		imshow("img", img);
		//cout<<img.size()<<endl;
		waitKey(33);
	}
	capture.release();
	return 0;
}
*/
//int main(void){
//cout<<getBuildInformation()<<endl;
//}

