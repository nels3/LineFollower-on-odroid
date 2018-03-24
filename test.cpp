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
int left_x = 60;
int right_x = 630;// 530;
int mask_y = 200;
int mask_width = 195;
int mask_y_1 = 300;
int mask_y_2 = 340;
int mask_x_l = 0;
int mask_x_r = 639;


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
	
	void edge(Mat &input, Mat &output, int threshold_value, int morph_size, int morph_elem) {
		Mat kernel_v, kernel_h , out_v, out_h, thresh, output_thresh;

		Point anchor;
		anchor = Point(-1, -1);

		kernel_v = Mat(1, 3, CV_32F);
		kernel_v.at<float>(0, 0) = -1;
		kernel_v.at<float>(0, 1) = 0;
		kernel_v.at<float>(0, 2) = 1;

		kernel_h = Mat(3, 1, CV_32F);
		kernel_h.at<float>(0, 0) = 1;
		kernel_h.at<float>(1, 0) = 0;
		kernel_h.at<float>(2, 0) = -1;

		threshold(input, thresh, threshold_value, 255, cv::THRESH_BINARY);

		int operation = 3;
		Mat element = getStructuringElement(morph_elem, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size));
		morphologyEx(thresh, output_thresh, operation, element);

		filter2D(output_thresh, out_v, -1, kernel_v, anchor, 0, cv::BORDER_DEFAULT);
		filter2D(output_thresh, out_h, -1, kernel_h, anchor, 0, cv::BORDER_DEFAULT);

		Mat output_edges = out_v;

		out_v.copyTo(output);

	}

	void mask(Mat &input, Mat &output) {
		Point mask_points[6] =
		{
			Point(mask_x_l, mask_y_2),
			Point(mask_x_l, mask_y_1),
			Point(left_x, mask_y),
			Point(right_x, mask_y),
			Point(mask_x_r, mask_y_1),
			Point(mask_x_r, mask_y_2)
		};
		Mat mask = Mat::zeros(cv::Size(640, 360), CV_8UC1);
		fillConvexPoly(mask, mask_points, 6, cv::Scalar(255, 0, 0));

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
		if (mode = 1){
			for (size_t i = 0; i < lf.detected_lines.size(); i++) {

				//cout << "lp:" << i << "\t"<<"rho"<<lf.detected_lines[i][0] << "\t theta" << lf.detected_lines[i][1] << endl;
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
			
			if (x_middle > 0 and x_middle < CAM_RES_X and y !=mask_y) {
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
	Mat img(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat mask(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat detected(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat gray(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat edge(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat frame(CAM_RES_Y, CAM_RES_X, CV_8UC4);

	vector<Mat> frame_split(4);
	
	//namedWindow("img", WINDOW_AUTOSIZE);
	namedWindow("value", WINDOW_AUTOSIZE);
	namedWindow("first", WINDOW_AUTOSIZE);

	int thresh_value = 160;
	int P_min_votes = 20;
	int P_max_gap = 20;
	int P_min_len = 20;


	createTrackbar("Thresh", "value", &thresh_value, 400,NULL);
	createTrackbar("Mask_x", "value", &right_x, 600, NULL);
	createTrackbar("Mask_y", "value", &left_x, 400, NULL);
	//createTrackbar("mask_y_1 ", "value", &mask_y_1, 600, NULL);
	//createTrackbar("mask_y_2", "value", &mask_y_2, 400, NULL);
	//createTrackbar("mask_x_r", "value", &mask_x_r, 400, NULL);

	
	VideoCapture capture = VideoCapture(0);
	if (cam == false) {
		//dane z pliku
		const string file_name = "lab21.jpg";

		//obraz wczytywany w szarosci
		img = imread(file_name, CV_LOAD_IMAGE_GRAYSCALE);
		if (!img.data) {
			cout << "Nie odnalezionu pliku " << file_name;
			return -1;
		}
	}

	
	while (true)
	{
		if (cam == true) {
			capture >> frame;
			split(img, frame_split); //frame
			img = frame_split[0];
		}
		

		//rozmywamy
		linefollower.blur(img, gray);
		
		//szukamy krawedzi
		linefollower.edge(gray, edge, thresh_value, 7, 0);

		//tworzymy maske
		linefollower.mask(edge, mask);

		linefollower.detectline(mask, detected);
		
		linefollower.findlines();

		linefollower.drawlines(detected);

		linefollower.findmiddle(img);

		linefollower.check_middle();
	

		//imshow("img", detected);
		imshow("first", img);
		waitKey(200);
	}
	capture.release();

	waitKey();
	return 0;
}
