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

//////variables//////
int whiteblackthreshold = 50;
int erodeIterations = 3;
int dilateIterations = 4;
int searchRadius = 80;
int searchPoints = 20;
std::string version = "0.1.0";
/////////////////////

std::vector<cv::Point>  findSurroundingPixels(cv::Mat image, cv::Point centerPixel,int radius, int pointAmount){
// Finds a circle of pixels around a center pixel. Returns an array of the resulting pixels
// image: the image to search in. Used to check if the pixel is in image bounds
// centerPixel: the central pixel around which the circle is drawn
// radius: the radius of the circle
// pointAmount: the amount of points on the circle

	std::vector<cv::Point> points; //create array to store the points
	for(int i = 0; i <= pointAmount; i++){ //iterate over the amount of points
		double angle = 2.0*M_PI*i/pointAmount; //calculate the angle of the point
		cv::Point point(centerPixel.x + radius * sin(angle),centerPixel.y - radius * cos(angle)); //calculate the point
		if((point.x <= image.size().width && point.x >= 0) && (point.y <= image.size().height && point.y >= 0)){ //check if the point is in the image bounds
			points.push_back(point); //add the point to the array
		}
	}
	return points; //return the array
}

void drawScanFlower(cv::Mat image, std::vector<cv::Point> centers){
// Draws a pattern of lines and a circle of dots around a center pixel
// image: the image to draw on
// centers: the center pixels around which the pattern is drawn

	for(int j = 0; j <= centers.size(); j++){ //iterate over the amount of center pixels
		std::vector<cv::Point> nextPixels = findSurroundingPixels(image, centers[j], searchRadius, searchPoints); //find the surrounding pixels and store them in an array
		for(int i = 0; i <= nextPixels.size(); i++){ //iterate over the amount of surrounding pixels
			line(image, centers[j], nextPixels[i], cv::Scalar(255, 255, 255), 1); //draw a line from the center pixel to the surrounding pixel
			circle(image, nextPixels[i], 2, cv::Scalar(255, 0, 0), -1); //draw a dot at the surrounding pixel
		}
	}
}

void cameraCalibration(std::string path){
//calculates radial distortion which can then be eliminated by calling cv::undistort(<input>, <output>, K, D);
//got this from here https://www.youtube.com/watch?v=FGqG1P36xxo&list=PL5B692fm6--ufBviUGK3hlwL1hVSyorZx&index=8
//path: the path to the calibration images

	cv::Mat image, image_gray, K, D; //create necessary matrixes
	std::vector<cv::Mat> Rs, Ts; //create necessary matrixes
	cv::VideoCapture calib(path); //load calibration images
    std::vector<cv::Point2f> corners_image;
    std::vector< std::vector<cv::Point2f> > points_image;
    cv::Size pattern_size(8, 6);
    for (;;){ //iterate over all calibration images
		if (!calib.read(image)){break;}	//check if any calibration images are left

		cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY); //convert all calibration images to grayscale
		bool found = cv::findChessboardCorners(image_gray, pattern_size, corners_image); //search for chessboard pattern
		if (found){ //if found, draw chessboard corners
			cv::drawChessboardCorners(image, pattern_size, corners_image, found);
			points_image.push_back(corners_image); //store the corners
		}
    }
	//do some math the indian guy told me to do
    std::vector< cv::Point3f > corners_world; //create necessary matrixes
    for(int i = 0; i < pattern_size.height; i++){ //iterate over the chessboard pattern
		for(int j = 0; j < pattern_size.width; j++){ //iterate over the chessboard pattern
			corners_world.push_back(cv::Point3f(j, i, 0)); //store the corners
		}
    }
    std::vector< std::vector<cv::Point3f> > points_world(points_image.size(), corners_world); //store the corners
    double rms = cv::calibrateCamera(points_world, points_image, image_gray.size(), K, D, Rs, Ts); //calibrate the camera with the resulting data
	std::cout << "Reprojection error: " << rms << std::endl;  //print reprojection error. closer to 1 = better calibration. note to self: take some better pictures
}

int main()
{
	std::cout << "Started line recognition. Running:" << version << std::endl; //print version number please refer to SemVer for versioning
	//create all matrixes
	cv::Mat cameraCapture, imageUndistorted, imageGrayscale, imageCropped, imageThresholded, imageThresholdedEroded, imageThresholdedDilated, fullImageThresholded;
    cameraCalibration("./left%d.png"); //calculates radial distortion which can then be eliminated by calling cv::undistort(<input>, <output>, K, D);

	//start video capture
	//cv::VideoCapture input(0, cv::CAP_V4L2);
    //cv::VideoCapture set(cv::CAP_PROP_FPS, 120);
    cv::VideoCapture input("/home/jetson/Desktop/TestImages/testImage%d.png");
	
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierachy;
	std::vector<cv::Point> centers;
	
	for(;;){
		if(!input.read(cameraCapture)){
			//throw std::invalid_argument("no camera image available"); //TODO: write some functioning error handling
			break;
		}
		cv::undistort(cameraCapture, imageUndistorted, K, D); //eliminate radial distortion
		cv::cvtColor(imageUndistorted, imageGrayscale, cv::COLOR_BGR2GRAY); //convert to grayscale
		imageCropped = imageGrayscale(cv::Range(imageGrayscale.size().height - 50, imageGrayscale.size().height),cv::Range(0,imageGrayscale.size().width)); //crop the lower part of the image to find the beginning of the line
		cv::threshold(imageCropped, imageThresholded, whiteblackthreshold, 255, cv::THRESH_BINARY_INV); //threshold the image to mask black parts of the image
		//cv::threshold(imageGrayscale, fullImageThresholded, whiteblackthreshold, 255, cv::THRESH_BINARY_INV);
		cv::erode(imageThresholded, imageThresholdedEroded, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10,10), cv::Point(-1, -1)), cv::Point(-1, -1), erodeIterations, cv::BORDER_DEFAULT); //erode the mask to remove noise
		cv::dilate(imageThresholdedEroded, imageThresholdedDilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10,10), cv::Point(-1, -1)), cv::Point(-1, -1), dilateIterations, cv::BORDER_DEFAULT); //dilate the mask to remove noise
		cv::findContours(imageThresholdedDilated, contours, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0,imageGrayscale.size().height - 50)); //find contours in the mask
		
		for(int i =0; i <= contours.size() && contours.size() > 0; i++){//iterate over the amount of contours
			cv::Point2f center; //create a point to store the center of the contour
			cv::Moments M = cv::moments(contours[i]); //calculate the moments of the contour
			if(M.m00 != 0){ //check if M.m00 is not 0 to avoid division by 0
				center.x = M.m10/M.m00;
				center.y = M.m01/M.m00;
				centers.push_back(center);
				cv::circle(imageUndistorted, center, 5, cv::Scalar(100, 0, 250), -1); //draw a circle at the center of the contour
			}/*
			else if(M.m00 = 0){
				center.x = imageUndistorted.size().width / 2;
				center.y = imageUndistorted.size().height;
				centers.push_back(center);
				cv::circle(imageUndistorted, center, 5, cv::Scalar(100, 0, 250), -1);
				std::cout << "divided by 0" << std::endl;
			}*/

			drawScanFlower(imageUndistorted, centers);
			
		}
		
		cv::drawContours(imageUndistorted, contours, -1, cv::Scalar(0, 0, 255), 2); //draw the contours to visualize the found line
				
		imshow("line full", imageUndistorted);
		imshow("line", imageThresholded);
		cv::waitKey(); //continue when a key is pressed
			
		//exit when esc key is pressed
		char c = cv::waitKey(1);
		if(c == 27){ break;} //27 = ESC key 
	}
}
