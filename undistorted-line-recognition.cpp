#define _USE_MATH_DEFINES
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
//#include <stdexcept>

int whiteblackthreshold = 50;
int erodeIterations = 3;
int dilateIterations = 4;
int searchRadius = 80;
int searchPoints = 20;

// Find the contour with the biggest area size
std::pair<int, int> findBiggestContour(std::vector<std::vector<cv::Point>> contours) {
	std::pair<int, int> c = std::make_pair(-1, 0);
	int currentArea;
	
	for(int i=0; i<contours.size(); i++) {
		currentArea = cv::contourArea(contours.at(i));
		if(currentArea > c.second) {
			c = std::make_pair(i, currentArea);
		}
	}
	return c;
}

std::vector<cv::Point>  findSurroundingPixels(cv::Mat image, cv::Point centerPixel,int radius, int pointAmount){
	std::vector<cv::Point> points;
	for(int i = 0; i <= pointAmount; i++){
		double angle = 2.0*M_PI*i/pointAmount;
		cv::Point point(centerPixel.x + radius * sin(angle),centerPixel.y - radius * cos(angle));
		if((point.x <= image.size().width && point.x >= 0) && (point.y <= image.size().height && point.y >= 0)){
			points.push_back(point);
		}
	}
	return points;
}

void drawScanFlower(cv::Mat image, std::vector<cv::Point> centers){
	for(int j = 0; j <= centers.size(); j++){
	std::vector<cv::Point> nextPixels = findSurroundingPixels(image, centers[j], searchRadius, searchPoints);
	for(int i = 0; i <= nextPixels.size(); i++){
		line(image, centers[j], nextPixels[i], cv::Scalar(255, 255, 255), 1);
		circle(image, nextPixels[i], 2, cv::Scalar(255, 0, 0), -1);
	}}
}

int main()
{
	//create all matrixes
	cv::Mat img, imgu, imgb, imgc, image, image_gray, K, D, undistorted, thresh, thresholdEroded, thresholdDilated, fullThreshold;
	std::vector<cv::Mat> Rs, Ts;
	
	
    cv::VideoCapture calib("./left%d.png");

    std::vector<cv::Point2f> corners_image;
    std::vector< std::vector<cv::Point2f> > points_image;
    cv::Size pattern_size(8, 6);
    
	
    for (;;){
		//check if any calibration images are left
		if (!calib.read(image))
			break;
			
		//convert all calibration images to grayscale
		cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
	
		//search for chessboard pattern
		bool found = cv::findChessboardCorners(image_gray, pattern_size, corners_image);
		//if found draw chessboard corners
		if (found){
			cv::drawChessboardCorners(image, pattern_size, corners_image, found);
			points_image.push_back(corners_image);
		}
    }

	//do some math the indian guy told me to do
    std::vector< cv::Point3f > corners_world;
    for(int i = 0; i < pattern_size.height; i++){
		for(int j = 0; j < pattern_size.width; j++){
			corners_world.push_back(cv::Point3f(j, i, 0));
		}
    }

    std::vector< std::vector<cv::Point3f> > points_world(points_image.size(), corners_world);

	//calibrate the camera with the resulting data
    double rms = cv::calibrateCamera(points_world, points_image, image_gray.size(), K, D, Rs, Ts);
    //print reprojection error. closer to 1 = better calibration. note to self: take some better pictures
	std::cout << "Reprojection error: " << rms << std::endl;
	
	//apply undistortion
    cv::undistort(image_gray, undistorted, K, D);
	
	//start video capture
	//cv::VideoCapture input(0, cv::CAP_V4L2);
    //cv::VideoCapture set(cv::CAP_PROP_FPS, 120);
    cv::VideoCapture input("../../../TestImages/testImage%d.png");
	
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierachy;
	std::vector<cv::Point> centers;
	
	for(;;){
		if(!input.read(img)){
			//throw std::invalid_argument("no camera image available");
			break;
		}
		
		cv::undistort(img, imgu, K, D);
		cv::cvtColor(imgu, imgb, cv::COLOR_BGR2GRAY);
		
		imgc = imgb(cv::Range(imgb.size().height - 50, imgb.size().height),cv::Range(0,imgb.size().width));
		
		cv::threshold(imgc, thresh, whiteblackthreshold, 255, cv::THRESH_BINARY_INV);
		cv::threshold(imgb, fullThreshold, whiteblackthreshold, 255, cv::THRESH_BINARY_INV);
		
		cv::erode(thresh, thresholdEroded, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10,10), cv::Point(-1, -1)), cv::Point(-1, -1), erodeIterations, cv::BORDER_DEFAULT);
	
		cv::dilate(thresholdEroded, thresholdDilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10,10), cv::Point(-1, -1)), cv::Point(-1, -1), dilateIterations, cv::BORDER_DEFAULT);
	
		//thresh = cv::Scalar(255, 255, 255) - thresh;

		cv::findContours(thresholdDilated, contours, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0,imgb.size().height - 50));
		
		for(int i =0; i <= contours.size() && contours.size() > 0; i++){
			cv::Point2f center;
			cv::Moments M = cv::moments(contours[i]);
			if(M.m00 != 0){
				center.x = M.m10/M.m00;
				center.y = M.m01/M.m00;
				centers.push_back(center);
				cv::circle(imgu, center, 5, cv::Scalar(100, 0, 250), -1);
			}/*
			else if(M.m00 = 0){
				center.x = imgu.size().width / 2;
				center.y = imgu.size().height;
				centers.push_back(center);
				cv::circle(imgu, center, 5, cv::Scalar(100, 0, 250), -1);
				std::cout << "divided by 0" << std::endl;
			}*/
			drawScanFlower(imgu, centers);
			
		}
		
		
		/*
		// Get a rectangle around the biggest contour
		std::pair<int, int> bigC = findBiggestContour(contours);
		// Falls eine Kontur gefunden wurde
		if(bigC.first != -1) {
			cv::RotatedRect minRect = cv::minAreaRect(contours.at(bigC.first));
			// 4 corner points of rectangle
			cv::Point2f box[4];
			minRect.points(box);
			// GET CENTER POINT
			cv::Point2f center = minRect.center;
			float ax = center.x;
			float ay = center.y;
			// Convert points to integer
			std::vector<cv::Point> intBox;
			for (int i=0; i<4; i++) {
				intBox.push_back(cv::Point(std::round(box[i].x), std::round(box[i].y)));
			}
			//Draws the rectangle
			cv::polylines(imgu, intBox, -1, cv::Scalar(0, 0, 255), 2);
		*/
		cv::drawContours(imgu, contours, -1, cv::Scalar(0, 0, 255), 2);
		
		
		
			imshow("line full", imgu);
			imshow("line", thresh);
			cv::waitKey();
			
			//exit when esc key is pressed
		char c = cv::waitKey(1);
		if(c == 27){ //27 = ESC key
			break;
		}
	
}}
