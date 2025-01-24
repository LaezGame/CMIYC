#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
//#include <stdexcept>

int whiteblackthreshold = 150;

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

int main()
{
	//create all matrixes
	cv::Mat img, imgu, imgb, imgc, image, image_gray, K, D, undistorted, thresh;
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
	cv::VideoCapture input(0, cv::CAP_V4L2);
    cv::VideoCapture set(cv::CAP_PROP_FPS, 120);
	
	cv::SimpleBlobDetector::Params params;
	
	params.minThreshold = 10;
	params.maxThreshold = 200;
	params.filterByArea = true;
	params.minArea = 1500;
	params.filterByCircularity = false;
	params.minCircularity = 0.1;
	params.filterByConvexity = false;
	params.minConvexity = 0.87;
	params.filterByInertia = true;
	params.minInertiaRatio = 0.01;
	
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierachy;
	
	for(;;){
		if(!input.read(img)){
			//throw std::invalid_argument("no camera image available");
			break;
		}
		
		cv::undistort(img, imgu, K, D);
		cv::cvtColor(imgu, imgb, cv::COLOR_BGR2GRAY);
		
		imgc = imgb(cv::Range(imgb.size().height - 50, imgb.size().height),cv::Range(0,imgb.size().width));
		
		cv::threshold(imgc, thresh, whiteblackthreshold, 200, cv::THRESH_BINARY_INV);
	
		//thresh = cv::Scalar(255, 255, 255) - thresh;

		cv::findContours(thresh, contours, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		
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
			
			imshow("line full", imgu);
			imshow("line", thresh);
		}
		
		//exit when esc key is pressed
		char c = cv::waitKey(1);
		if(c == 27){ //27 = ESC key
			break;
		}
	}
}