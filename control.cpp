#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include <string>
#include <iostream>

#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <time.h>
#include <thread>
#include <mutex>
#include <atomic>

#define CAM_RES_X 640
#define CAM_RES_Y 360

using namespace std;
using namespace cv;

int mode = 1;
float y_cmd = 100;
bool cam = false;

//mask
int maskx1 = 0;
int masky1 = 340;
int maskx2 = 00;
int masky2 = 100;
int maskx3 = 640;
int masky3 = masky2;
int maskx4 = CAM_RES_X;
int masky4 = 340;


class linefollower_values {
public:
	vector<Vec2f> detected_lines;
	vector<Vec4i> detected_prob;
	float rho_left, theta_left, rho_right, theta_right;
	float middle;
	float offset;
	int counter_of_lines = 0;
};

linefollower_values lf;

class linefollower_data {
public:
	void blur(Mat &input, Mat &output) {
		GaussianBlur(input, output, Size(5, 5), 0, 0);
		
	}

	void edge(Mat &input, Mat &output, int thresh) {
		Mat kernel, thresh_out;
		kernel = Mat(1, 3, CV_32F);
		kernel.at<float>(0, 0) = -1;
		kernel.at<float>(0, 1) = 0;
		kernel.at<float>(0, 2) = 1;

		threshold(input, thresh_out, thresh, 255, THRESH_BINARY);

		Mat element = getStructuringElement(0, Size(15, 15), Point(7, 7));
		morphologyEx(thresh_out, thresh_out, 3, element);

		filter2D(thresh_out, output, -1, kernel, Point(-1, -1), 0, BORDER_DEFAULT);
	}

	void mask(Mat &input, Mat &output) {
		masky3 = masky2;
		Point mask_points[6] =
		{
			Point(maskx1, masky1),
			Point(maskx2, masky2),
			Point(maskx3, masky3),
			Point(maskx4, masky4)
		};
		Mat mask = Mat::zeros(cv::Size(640, 360), CV_8UC1);
		fillConvexPoly(mask, mask_points, 4, cv::Scalar(255, 0, 0));
		//imshow("img", mask);
		bitwise_and(input, mask, output);
	}

	void detectline(Mat &input, Mat &output) {
		if (mode = 2) {
			HoughLinesP(input, lf.detected_prob, 1, CV_PI / 180, 50, 50, 10);
			for (size_t i = 0; i < lf.detected_prob.size(); i++)
			{
				Vec4i l = lf.detected_prob[i];
				line(output, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
			}
		}
		if (mode = 1) {
			HoughLines(input, lf.detected_lines, 1, CV_PI / 180, 50);
		}
	}

	void findlines() {
		float rho_left = 0, theta_left = 0;
		float rho_right = 0, theta_right = 0;
		bool right = false;
		bool left = false;
		if (mode = 1) {
			for (size_t i = 0; i < lf.detected_lines.size(); i++) {
				float rho = lf.detected_lines[i][0];
				float theta = lf.detected_lines[i][1];
				if (rho > 0 and abs(rho_left - rho) > 50) {
					rho_left = rho;
					theta_left = theta;
					left = true;
				}
				if (rho < 0 and abs(rho_right - rho)>50) {
					rho_right = rho;
					theta_right = theta;
					right = true;
				}
			}
			if (left == true and right == true) {
				lf.counter_of_lines = 2;
				lf.theta_right = theta_right;
				lf.theta_left = theta_left;
				lf.rho_left = rho_left;
				lf.rho_right = rho_right;
			}
			else if (right == true and left == false) {
				lf.counter_of_lines = 3;
				lf.rho_right = rho_right;
				lf.theta_right = theta_right;
			}
			else if (left == true and right == false) {
				lf.counter_of_lines = 1;
				lf.theta_left = theta_left;
				lf.rho_left = rho_left;
			}
		}
		else if (mode = 2) {
			cout << "wykrylem" << lf.detected_prob.size();
		}
	}

	void drawlines(Mat &output) {
		Point pt1, pt2;
		double a = cos(lf.theta_left), b = sin(lf.theta_left);
		double x0 = a * lf.rho_left, y0 = b * lf.rho_left;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(output, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
		a = cos(lf.theta_right), b = sin(lf.theta_right);
		x0 = a * lf.rho_right, y0 = b * lf.rho_right;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(output, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
	}

	void findmiddle(Mat &output) {
		Point pt1, pt2;
		float x_middle;
		float x_right;
		float x_left;
		float x_middle_prev;
		int mask_y = int(masky3);
		for (int y = mask_y; y < CAM_RES_Y; y = y + 20) {
			if (y != mask_y) {
				x_middle_prev = x_middle;
			}
			x_left = -sin(lf.theta_left) / cos(lf.theta_left)*y + lf.rho_left / cos(lf.theta_left);
			x_right = -sin(lf.theta_right) / cos(lf.theta_right)*y + lf.rho_right / cos(lf.theta_right);
			x_middle = (x_left + x_right) / 2;
			if (y == mask_y + 3 * 20) {
				lf.middle = x_middle;
			}

			if (x_middle > 0 and x_middle < CAM_RES_X and y != mask_y) {
				pt1.x = x_middle_prev;
				pt1.y = y - 10;
				pt2.x = x_middle;
				pt2.y = y + 10;
				line(output, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
			}
		}
	}

	void check_middle() {
		lf.offset = lf.middle - CAM_RES_X / 2;
		cout << lf.offset << endl;
	}
};

linefollower_data linefollower;

int main()
{
	vector<Mat> frame_split(4);
	Mat frame(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat img(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat img2(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat img_gray(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	Mat mask(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat display(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat gray(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat edge(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	
	namedWindow("img", WINDOW_AUTOSIZE);
	namedWindow("value", WINDOW_AUTOSIZE);
	//namedWindow("first", WINDOW_AUTOSIZE);

	int thresh_value = 158;

	createTrackbar("Thresh", "value", &thresh_value, 300, NULL);
	createTrackbar("lewy_dolny", "value", &maskx1, 300, NULL);
	createTrackbar("mask_x2", "value", &maskx2, 400, NULL);
	createTrackbar("wysokosc", "value", &masky2, 340, NULL);
	createTrackbar("mask_x3", "value", &maskx3, 640, NULL);
	
	VideoCapture capture;
	capture.open(0, CAP_V4L2);
	if (cam == false) {
		//dane z pliku
		const string file_name = "15.jpg";
		img = imread(file_name);
		if (!img.data) {
			cout << "Nie odnalezionu pliku " << file_name;
			return -1;
		}		
		cvtColor(img, img2, CV_BGR2GRAY);
	}
	else {
		if (!capture.isOpened()) {
			cout << "error with camera" << endl;
		}
		else {
			capture.set(CAP_PROP_FRAME_WIDTH, 640);
			capture.set(CAP_PROP_FRAME_HEIGHT, 360);
			capture.set(CAP_PROP_CONVERT_RGB, false);
			cout << "MODE" << capture.get(CAP_PROP_MODE) << endl;
			cout << "RGB" << capture.get(CAP_PROP_CONVERT_RGB) << endl;
		}
	}

	while (true){
		if (cam == true) {
			capture >> frame;
			cv::split(frame, frame_split);
			img_gray = frame_split[0];
		}
		//rozmywamy
		if (cam == false) 
			linefollower.blur(img2, gray);
		else 
			linefollower.blur(img_gray, gray);

		//szukamy krawedzi
		linefollower.edge(gray, edge, thresh_value);
				
		//tworzymy maske
		linefollower.mask(edge, mask);
			
		Mat detected = Mat::zeros(CAM_RES_Y, CAM_RES_X, CV_8UC4);

		linefollower.detectline(mask, detected);

		linefollower.findlines();
		cvtColor(img2, display, 9);

		if (cam == false) {
			linefollower.drawlines(display);
			linefollower.findmiddle(display);
		}
		else {
			cvtColor(frame, display, CV_YUV2RGB);
			linefollower.drawlines(display);
			linefollower.findmiddle(display);
		}
		linefollower.check_middle();

		
		
		if (cam == false){
			int a = (int)lf.offset;
			stringstream ss;
			ss << a;
			string str = ss.str();
			string tex = ("Offset = ");
			tex.append(str);
			putText(img, tex, Point(CAM_RES_X/2-50,CAM_RES_Y/4),1,1,Scalar(255,0,0));
			imshow("first", display);
		}
		else{
			imshow("first", display);
		}
		//imshow("img", display);
		waitKey(200);
		
	}
	capture.release();

	waitKey();
	return 0;
}
