#define _USE_MATH_DEFINES
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <unistd.h>       // Used for UART
#include <sys/fcntl.h>    // Used for UART
#include <termios.h>      // Used for UART
#include <string>
#include <cstring>
#include <errno.h>
//#include <stdexcept>

//////variables//////
int whiteblackthreshold = 50;
int pathDepth = 2;
int erodeIterations = 3;
int dilateIterations = 4;
int searchRadius = 120;
int searchPoints = 15;
std::string version = "0.1.1";
float angleThresh = 0.1;
/////////////////////

std::vector<cv::Point>  findSurroundingPixels(cv::Mat image, cv::Point centerPixel,int radius, int pointAmount){
// Finds a circle of pixels around a center pixel. Returns an array of the resulting pixels
// image: the image to search in. Used to check if the pixel is in image bounds
// centerPixel: the central pixel around which the circle is drawn
// radius: the radius of the circle
// pointAmount: the amount of points on the circle

	std::vector<cv::Point> points; //create array to store the points
	for(int i = 0; i < pointAmount; i++){ //iterate over the amount of points
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

	for(int j = 0; j < centers.size(); j++){ //iterate over the amount of center pixels
		std::vector<cv::Point> nextPixels = findSurroundingPixels(image, centers[j], searchRadius, searchPoints); //find the surrounding pixels and store them in an array
		for(int i = 0; i < nextPixels.size(); i++){ //iterate over the amount of surrounding pixels
			line(image, centers[j], nextPixels[i], cv::Scalar(255, 255, 255), 1); //draw a line from the center pixel to the surrounding pixel
			circle(image, nextPixels[i], 2, cv::Scalar(255, 0, 0), -1); //draw a dot at the surrounding pixel
		}
	}
}

std::vector<cv::Mat> cameraCalibration(std::string path){
//calculates radial distortion which can then be eliminated by calling cv::undistort(<input>, <output>, K, D);
//got this from here https://www.youtube.com/watch?v=FGqG1P36xxo&list=PL5B692fm6--ufBviUGK3hlwL1hVSyorZx&index=8
//path: the path to the calibration images

	cv::Mat image, image_gray, K, D; //create necessary matrixes
	std::vector<cv::Mat> Rs, Ts; //create necessary matrixes
	cv::VideoCapture calib(path); //load calibration images
    std::vector<cv::Point2f> corners_image;
    std::vector< std::vector<cv::Point2f> > points_image;
    cv::Size pattern_size(8, 6);
	std::vector<cv::Mat> calibrationData; //create necessary matrixes
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
	calibrationData.push_back(K); //store the calibration data
	calibrationData.push_back(D); //store the calibration data
	return calibrationData; //return the calibration data
}

std::Pair<cv::Mat, cv::Point> findPath(int repetitions, int searchRadius, cv::Point startpoint, cv::Mat image, cv::Mat outputImage){
	std::vector<cv::Point> nextPixels; //find the surrounding pixels and store them in an array
	std::vector<cv::Point> validPixels, goingToCheck, homePoint;
	cv::Point lastPoint = startpoint;
	homePoint.push_back(startpoint);

	nextPixels = findSurroundingPixels(image, startpoint, searchRadius, searchPoints);

	for (int i = 0; i < searchPoints; i++){ //iterate over the amount of surrounding pixels
		if(nextPixels[i].x > 0 && nextPixels[i].x < image.cols - 1 && nextPixels[i].y > 0 && nextPixels[i].y < image.rows - 1 && image.at<uchar>(nextPixels[i]) == 255){ //check if the pixel is white and within bounds, excluding the perimeter
			line(outputImage, startpoint, nextPixels[i], cv::Scalar(0, 255, 0), 1); //draw a line from the startpoint to the surrounding pixel
			goingToCheck.push_back(nextPixels[i]); //add the surrounding pixel to the array
			lastPoint = nextPixels[i];  // Update last point found
		}
	}

	for(int i = 0; i < repetitions; i++){ //iterate over the amount of repetitions
		for(int j = 0; j < goingToCheck.size(); j++){
			nextPixels = findSurroundingPixels(image, goingToCheck[j], searchRadius, searchPoints); //find the surrounding pixels and store them in an array
			for(int k = 0; k < nextPixels.size(); k++){ //iterate over the amount of surrounding pixels
				if(nextPixels[k].x > 0 && nextPixels[k].x < image.cols - 1 && nextPixels[k].y > 0 && nextPixels[k].y < image.rows - 1 && image.at<uchar>(nextPixels[k]) == 255){ //check if the pixel is white and within bounds, excluding the perimeter
					line(outputImage, goingToCheck[j], nextPixels[k], cv::Scalar(0, 255, 0), 1); //draw a line from the startpoint to the surrounding pixel
					validPixels.push_back(nextPixels[k]); //add the surrounding pixel to the array
					lastPoint = nextPixels[k];  // Update last point found
				}
			}
			nextPixels.clear(); //clear the array
		}
		goingToCheck = validPixels;
		validPixels.clear();
	}
	return {outputImage, lastPoint}; //return the image and the last point
}

cv::Mat convertImage(cv::Mat image, cv::Range cropHeight, cv::Range cropWidth){
	cv::Mat image_gray, image_thresholded, image_thresholded_eroded, image_thresholded_dilated;

	image = image(cropHeight, cropWidth);

	cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY); //convert to grayscale
	cv::threshold(image_gray, image_thresholded, whiteblackthreshold, 255, cv::THRESH_BINARY_INV); //threshold the image to mask black parts of the image
	cv::erode(image_thresholded, image_thresholded_eroded, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10,10), cv::Point(-1, -1)), cv::Point(-1, -1), erodeIterations, cv::BORDER_DEFAULT); //erode the mask to remove noise
	cv::dilate(image_thresholded_eroded, image_thresholded_dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10,10), cv::Point(-1, -1)), cv::Point(-1, -1), dilateIterations, cv::BORDER_DEFAULT); //dilate the mask to remove noise
	return image_thresholded_dilated;
}

void uart_setup(int &fid) {
    // SETUP SERIAL WORLD
    struct termios  port_options;   // Create the structure

    tcgetattr(fid, &port_options);	// Get the current attributes of the Serial port


    //------------------------------------------------
    //  OPEN THE UART
    //------------------------------------------------
    // The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR   - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //	    O_NDELAY / O_NONBLOCK (same function)
    //               - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //                 if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //				   immediately with a failure status if the output can't be written immediately.
    //                 Caution: VMIN and VTIME flags are ignored if O_NONBLOCK flag is set.
    //	    O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.fid = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

    fid = open(uart_target, O_RDWR | O_NOCTTY );
    
    if (fid == -1)
    {
        std::cerr << "Error - Unable to open UART.  Ensure it is not in use by another application\n";
    }

    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);

    usleep(1000);  // 1 sec delay

    //------------------------------------------------
    // CONFIGURE THE UART
    //------------------------------------------------
    // flags defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html
    //	Baud rate:
    //         - B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200,
    //           B230400, B460800, B500000, B576000, B921600, B1000000, B1152000,
    //           B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE: - CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD  - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL  - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)

    port_options.c_cflag &= ~PARENB;            // Disables the Parity Enable bit(PARENB)
    port_options.c_cflag &= ~CSTOPB;            // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
    port_options.c_cflag &= ~CSIZE;	            // Clears the mask for setting the data size
    port_options.c_cflag |=  CS8;               // Set the data bits = 8
    port_options.c_cflag &= ~CRTSCTS;           // No Hardware flow Control
    port_options.c_cflag |=  CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines
    port_options.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both input & output
    port_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode
    port_options.c_oflag &= ~OPOST;                           // No Output Processing

    port_options.c_lflag = 0;               //  enable raw input instead of canonical,

    port_options.c_cc[VMIN]  = VMINX;       // Read at least 1 character
    port_options.c_cc[VTIME] = 10;           // Wait one second

    cfsetispeed(&port_options,BAUDRATE);    // Set Read  Speed
    cfsetospeed(&port_options,BAUDRATE);    // Set Write Speed

    // Set the attributes to the termios structure
    int att = tcsetattr(fid, TCSANOW, &port_options);

    if (att != 0 ) {
        std::cerr << "\nERROR in Setting port attributes" << std::endl;
    }

    // Flush Buffers
    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);
}

bool write_serial(int fid, std::string msg) {
    unsigned char tx_buffer[msg.size()];
    unsigned char *p_tx_buffer;

    p_tx_buffer = &tx_buffer[0];

    for(char i : msg) {
        *p_tx_buffer++ = i;
    }

    if (fid != -1)
    {
        int count = write(fid, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
        usleep(1000);   // .001 sec delay
        if (count < 0)  return false; // write failed
    }
    return true;
}

int main()
{
	std::cout << "Started line recognition. Running:" << version << std::endl; //print version number please refer to SemVer for versioning
	cv::Mat cameraCapture, imageUndistorted, foundPath, convertedImage;//create all matrixes
    std::vector<cv::Mat> calibrationData =  cameraCalibration("./left%d.png"); //calculates radial distortion which can then be eliminated by calling cv::undistort(<input>, <output>, K, D);
	cv::Mat K = calibrationData[0]; //get the calibration data
	cv::Mat D = calibrationData[1]; //get the calibration data

	//start video capture
	//cv::VideoCapture input(0, cv::CAP_V4L2);
    //cv::VideoCapture set(cv::CAP_PROP_FPS, 120);
    cv::VideoCapture input("/home/jetson/Desktop/TestImages/testImage%d.png");
	
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierachy;
	std::vector<cv::Point> centers;
	cv::Point lastFoundPoint;
	
	// Setup Serial Communication
	int fid= -1;
    uart_setup(fid);
    
    if (fid == -1) {
        std::cerr << "Error - Unable to open UART.  Ensure it is not in use by another application\n";
    }
    usleep(500000);   // 0.5 sec delay
	
	for(;;){
		if(!input.read(cameraCapture)){
			//throw std::invalid_argument("no camera image available"); //TODO: write some functioning error handling
			break;
		}
		centers.clear();

		
		cv::undistort(cameraCapture, imageUndistorted, K, D); //eliminate radial distortion

		convertedImage = convertImage(imageUndistorted, cv::Range(imageUndistorted.size().height - 50, imageUndistorted.size().height), cv::Range(0, imageUndistorted.size().width));
		cv::findContours(convertedImage, contours, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0,imageUndistorted.size().height - 50)); //find contours in the mask
		
		for(int i =0; i < contours.size() && contours.size() > 0; i++){//iterate over the amount of contours
			cv::Point2f center; //create a point to store the center of the contour
			cv::Moments M = cv::moments(contours[i]); //calculate the moments of the contour
			if(M.m00 != 0){ //check if M.m00 is not 0 to avoid division by 0
				center.x = M.m10/M.m00;
				center.y = M.m01/M.m00;
				centers.push_back(center);
				cv::circle(imageUndistorted, center, 5, cv::Scalar(100, 0, 250), -1); //draw a circle at the center of the contour
			}
		}

		drawScanFlower(imageUndistorted, centers);
		
		cv::drawContours(imageUndistorted, contours, -1, cv::Scalar(0, 0, 255), 2); //draw the contours to visualize the found line
				
		for(int i = 0; i < centers.size(); i++){ //iterate over the amount of center pixels
			[foundPath, lastFoundPoint] = findPath(pathDepth, searchRadius, centers[i], convertImage(imageUndistorted, cv::Range(0, imageUndistorted.size().height), cv::Range(0, imageUndistorted.size().width)), imageUndistorted); //find the path of the line
			if(foundPath.empty()){
				foundPath = imageUndistorted;
			}
			// Find the angle between start and end in relation to a vertical line
			float angle = cv::atan2(centers[i].y - lastFoundPoint.y, centers[i].x - lastFoundPoint.x) / cv::CV_PI;
			// between 0 and 1 go left, between 1 and 2 go right
			if (angle >= 0 && angle <= 1 - angleThresh) {
				std::cout << serial_write(fid, "R2:" << 255-angle*255 << ":L1:" << 255-angle*255 << "\n") << angle << std::endl; // turn left
			} else if (angle >= 1 + angleThresh && angle <= 2) {
				std::cout << serial_write(fid, "R1:" << (angle-1)*255 << ":L2:" << angle(-1)*255 << "\n") << angle << std::endl; // turn right
			} else if (angle > 1 - angleThresh && angle < 1 + angleThresh) {
				std::cout << serial_write(fid, "R2:" << 255 << ":L2:" << 255 << "\n") << angle << std::endl; // go forward
			} else {
				std::cout << "angle: " << angle << std::endl;
			}
		}

		if (!imageUndistorted.empty()) {
			imshow("line full", imageUndistorted);
		} else {
			std::cerr << "Error: imageUndistorted is empty" << std::endl;
		}
		if (!convertedImage.empty()) {
			imshow("line", convertedImage);
		} else {
			std::cerr << "Error: convertedImage is empty" << std::endl;
		}
		cv::waitKey(); //continue when a key is pressed
			
		//exit when esc key is pressed
		char c = cv::waitKey(1);
		if(c == 27){ break;} //27 = ESC key 
	}
}
