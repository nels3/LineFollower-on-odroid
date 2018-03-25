#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include <string>
#include <iostream>
using namespace std;
using namespace cv;
int counter = 0;
int main() {
	VideoCapture capture = VideoCapture(0);
	namedWindow("img", CV_WINDOW_AUTOSIZE);
	Mat frame, img, gray;


	while (true) {
		capture >> frame;
		frame.copyTo(img);
		//cvtColor(img, gray, CV_BGR2HSV);
		imshow("img", img);
	
		waitKey(100);
	}
	capture.release();
	return 0;
}