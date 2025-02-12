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
// a comment
// for UART
// Define Constants
const char *uart_target = "/dev/ttyUSB0";
#define     NSERIAL_CHAR   256
#define     VMINX          1
#define     MSG_LEN        15
#define     BAUDRATE       B9600

//////variables//////
int whiteblackthreshold = 50;
int pathDepth = 10;
int erodeIterations = 3;
int dilateIterations = 4;
int searchRadius = 50;
int searchPoints = 40;
std::string version = "0.2.0";
double lengthWeightModifier = 1.0;
double angleWeightModifier = 1.0;
float angleThresh = 0.01;
float borderThresh = 0.01;
double relativePosition = 0.5;
/////////////////////

std::vector<cv::Point> getOrderedSurroundingPoints(cv::Mat image, cv::Point centerPixel, int radius, int pointAmount) {
    std::vector<cv::Point> points; // create array to store the points

    for (int i = 0; i < pointAmount; i++) { // iterate over the amount of points
        double angle = 2.0 * M_PI * i / pointAmount; // calculate the angle of the point
        cv::Point point(centerPixel.x + radius * sin(angle), centerPixel.y - radius * cos(angle)); // calculate the point
        if ((point.x >= 0 && point.x < image.size().width) && (point.y >= 0 && point.y < image.size().height)) { // check if the point is in the image bounds
            points.push_back(point); // add the point to the array
        }
    }

    // Sort points from top to bottom
    std::sort(points.begin(), points.end(), [](const cv::Point& a, const cv::Point& b) {
        return a.y < b.y || (a.y == b.y && a.x < b.x);
    });

    return points; // return the array
}

void drawScanFlower(cv::Mat image, std::vector<cv::Point> centers) {
    // Draws a pattern of lines and a circle of dots around a center pixel
    // image: the image to draw on
    // centers: the center pixels around which the pattern is drawn

    for (int j = 0; j < centers.size(); j++) { // iterate over the amount of center pixels
        std::vector<cv::Point> nextPixels = getOrderedSurroundingPoints(image, centers[j], searchRadius, searchPoints); // get the surrounding pixels in a specific order
        for (int i = nextPixels.size() - 1; i >= 0; i--) { // iterate over the amount of surrounding pixels in reverse order
            line(image, centers[j], nextPixels[i], cv::Scalar(255, 255, 255), 1); // draw a line from the center pixel to the surrounding pixel
            int blueIntensity = 255 - (i * (255 / nextPixels.size())); // calculate decreasing blue tone
            circle(image, nextPixels[i], 2, cv::Scalar(255, 0, blueIntensity), -1); // draw a dot at the surrounding pixel with decreasing blue tone
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

double calculateDistance(const cv::Point& p1, const cv::Point& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double calculateAngle(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3) {
    double angle1 = atan2(p2.y - p1.y, p2.x - p1.x);
    double angle2 = atan2(p3.y - p2.y, p3.x - p2.x);
    return fabs(angle1 - angle2);
}

bool pointIsInImage(const cv::Point& point, const cv::Mat& image) {
	if(point.x > 0 && point.x < image.cols - 1 && point.y > 0 && point.y < image.rows - 1) return true;
	else return false;	
}

std::vector<std::vector<cv::Point>> getBranchedPoints(const std::vector<cv::Point>& points, int searchRadius, cv::Mat image, int repetitions) {
    std::vector<std::vector<cv::Point>> branchedPoints;
    double maxAngle = 3 * M_PI / 6; // Maximum angle in radians (90 degrees)

    for (const auto& point : points) {
        std::deque<std::vector<cv::Point>> currentBranches = {{point}};
		for (int r = 0; r < repetitions; r++) {
			std::deque<std::vector<cv::Point>> newBranches;
			for (const auto& branch : currentBranches) {
				const cv::Point& lastPoint = branch.back();
				std::vector<cv::Point> nextPixels = getOrderedSurroundingPoints(image, lastPoint, searchRadius, searchPoints);

				for (const auto& nextPoint : nextPixels) {
					if (pointIsInImage(nextPoint, image) && image.at<uchar>(nextPoint) == 255) {
						if (std::find(branch.begin(), branch.end(), nextPoint) == branch.end()) {
							if (branch.size() > 1) {
								double angle = calculateAngle(branch[branch.size() - 2], lastPoint, nextPoint);
								if (angle > maxAngle) {
									continue; // Skip points that do not meet the angle criteria
								}
							}
							std::vector<cv::Point> newBranch = branch;
							newBranch.push_back(nextPoint);
							newBranches.push_back(std::move(newBranch));
						}
					}
				}
			}

			// Discard branches that aren't promising
			if (newBranches.size() > 50) {
			std::nth_element(newBranches.begin(), newBranches.begin() + 50, newBranches.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
				double distanceA = calculateDistance(a.front(), a.back());
				double distanceB = calculateDistance(b.front(), b.back());
				return (distanceA / a.size()) > (distanceB / b.size());
			});
			newBranches.resize(50);
			}

			currentBranches = std::move(newBranches);
		}

        for (const auto& branch : currentBranches) {
            if (branch.size() > 1) {
                branchedPoints.push_back(std::move(branch));
            }
        }
    }
/*
    // Filter out branches that are too close to each other
    std::vector<std::vector<cv::Point>> filteredBranchedPoints;
    for (const auto& branch : branchedPoints) {
        bool tooClose = false;
        for (const auto& filteredBranch : filteredBranchedPoints) {
            if (calculateDistance(branch.back(), filteredBranch.back()) < searchRadius) {
                tooClose = true;
                break;
            }
        }
        if (!tooClose) {
            filteredBranchedPoints.push_back(branch);
        }
    }
*/
    return branchedPoints;
}


std::vector<double> checkBranchStraightnessAndLength(const std::vector<std::vector<cv::Point>>& branchedPoints) {
    std::vector<std::pair<double, double>> branchMetrics; // pair of <length, straightness>
    std::vector<double> branchGrade;

    for (const auto& branch : branchedPoints) {
        double totalLength = 0;
        double totalAngle = 1.0;
        double upwardScore = 1.0;
        double straightnessScore = 1.0;
        double distanceToRoot = 0.0;
/*
        for (size_t i = 1; i < branch.size(); ++i) {
            totalLength += calculateDistance(branch[i - 1], branch[i]);
            if (i > 1) {
                double angle = calculateAngle(branch[i - 2], branch[i - 1], branch[i]);
                totalAngle += angle;
                if (angle < M_PI / 4) { // Prefer branches with smaller angles
                    straightnessScore = straightnessScore * (1/angle);
                }
            }
            if (branch[i].y < branch[i - 1].y) { // Check if the current point is higher up than the previous point
                upwardScore += 1.0;
            }
        }
*/
        distanceToRoot = calculateDistance(branch.front(), branch.back());
        double averageAngle = (branch.size() > 2) ? totalAngle / (branch.size() - 2) : 0;
        branchMetrics.push_back(std::make_pair(totalLength, averageAngle));

        double grade = distanceToRoot /  branch.size();
        branchGrade.push_back(grade);
        std::cout << "Branch Length: " << totalLength << std::endl;
        std::cout << "Straightness Score: " << straightnessScore << std::endl;
        std::cout << "Upward Score: " << upwardScore << std::endl;
        std::cout << "Distance to Root: " << distanceToRoot << std::endl;
        std::cout << "Grade: " << grade << std::endl;
    }

    return branchGrade;
}

cv::Point drawBranchWithGrade(cv::Mat& image, const std::vector<cv::Point>& branch, double grade, int colorIntensity) {
    cv::Point lastFoundPoint;
    for (size_t i = 1; i < branch.size(); ++i) {
        line(image, branch[i - 1], branch[i], cv::Scalar(0, colorIntensity, 0), 2); // draw the branch
    }
    if (!branch.empty()) {
        cv::putText(image, std::to_string(grade), branch.back(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1); // add the grade
    }
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

    do {
		fid = open(uart_target, O_RDWR | O_NOCTTY );
		if (fid == -1) {
			switch (errno) {
				case ENOENT:
					std::cout << "Error: Device not found " << uart_target << std::endl;
					break;
				case EACCES:
					std::cout << "Error: Permission denied" << std::endl;
					break;
				case EBUSY:
					std::cout << "Error: Device busy" << uart_target << std::endl;
					break;
				default:
					std::cout << "Error: Failed to open device " << uart_target << " " << strerror(errno) << std::endl;
					break;
			}
		}
		usleep(1000000);
	} while (fid == -1);

    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);

    usleep(1000);  // 0.001 sec delay

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
        //usleep(1000);   // .001 sec delay
        if (count < 0)  return false; // write failed
    }
    return true;
}

std::vector<cv::Point> findContourCenters(const std::vector<std::vector<cv::Point>>& contours, cv::Mat image){
	std::vector<cv::Point> centers;
	for(int i =0; i < contours.size() && contours.size() > 0; i++){//iterate over the amount of contours
			cv::Point2f center; //create a point to store the center of the contour
			cv::Moments M = cv::moments(contours[i]); //calculate the moments of the contour
			if(M.m00 != 0){ //check if M.m00 is not 0 to avoid division by 0
				center.x = M.m10/M.m00;
				center.y = M.m01/M.m00;
				centers.push_back(center);
				cv::circle(image, center, 5, cv::Scalar(100, 0, 250), -1); //draw a circle at the center of the contour
			}
		}
	return centers;
}

int drawFPS(cv::Mat image){
	int fps = 1.0 / cv::getTickFrequency() * cv::getTickCount();
	std::string fpsString = "FPS: " + std::to_string(fps);
	cv::putText(image, fpsString, cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
	std::cout << fpsString << std::endl;
	return fps;
}

			//std::string writeString = "R2:" + std::to_string(0.5 + relativePosition * 200 * 2) + "\nL2:" + std::to_string(0.5 - relativePosition * 200 * 2) + "\n";
std::string motorString(int rSpeed, int lSpeed) {
	std::string rDirection;
	std::string lDirection;
	
	if(rSpeed > 0){
		rDirection = "2";
	}  else if(rSpeed < 0){
		rDirection = "-1";
	} else if(rSpeed == 0){
		rDirection = "1";
	}
	if(lSpeed > 0){
		lDirection = "2";
	}  else if(lSpeed < 0){
		lDirection = "-1";
	} else if(lSpeed == 0){
		lDirection = "1";
	}


	std::string writeString = "R" + rDirection + ":" + std::to_string(abs(rSpeed)) + "\nL" + lDirection + ":" + std::to_string(abs(lSpeed)) + "\n";
	return writeString;
}

void sendMotorData(int rSpeed, int lSpeed, int fid){
	std::string writeString = motorString(rSpeed, lSpeed);
	std::cout << write_serial(fid, writeString) << std::endl;
}

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

char findCrossing(cv::Mat &dImg, std::vector<cv::Point> bestBranch) {
	std::vector<std::vector<cv::Point>> tContours, hContours;
	bool intersects = false;
	int leftCount = 0, rightCount = 0;
	
	// Define ROI (top part of the image)
    cv::Rect roi(0, 0, frame.cols, frame.rows * 0.3);
    cv::Mat dImgTop = frame(roi);
	
	//Find the Contours in top part of the image
	cv::findContours(dImgTop, tContours, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE); //find contours in the top 30% of the mask
	
	for (const std::vector<cv::Point>& contour : tContours) {
        cv::Rect boundingBox = cv::boundingRect(contour);
        float aspectRatio = static_cast<float>(boundingBox.width) / boundingBox.height;

        // Filter for horizontal lines based on aspect ratio and width (at least a fourth of image) threshold
        if (aspectRatio > 3.0 && boundingBox.width > dImg.rows * 0.25) {
            cv::rectangle(dImg, boundingBox, cv::Scalar(255, 255, 255), 2);  // Draw rectangle for visualization
            hContours.push_back(contour);
        }
    }
    
    // Get bounding box for the biggest contour
    int biggestHContourIdx = findBiggestContour(hContours).first;
    cv::Rect hBoundingBox = cv::boundingRect(hCotours[biggestHContourIdx]);

	for (const auto& point : verticalPathPoints) {
		if (hBoundingBox.contains(point)) {
			intersects = true;
		}
		// Check if the point is to the left or right of the vertical path's center
		if (point.x < hBoundingBox.x + hBoundingBox.width / 2) {
			leftCount++;
		} else {
			rightCount++;
		}
	}
	
	// Step 3: Analyze the results
	/*if (intersects) {
		return 'i'; // intersection to both sides, use preprogrammed direction
	}*/
	
	if (leftCount > 0 && rightCount > 0) {
		return 'i'; // intersection to both sides, use preprogrammed direction
	} else if (leftCount > 0) {
		return 'l'; // intersection to the left, go left
	} else if (rightCount > 0) {
		return 'r'; // intersection to the right, go right
	}
	
	return 'n'; // do nothing
}

int main()
{
	std::cout << "Started line recognition. Running:" << version << std::endl; //print version number please refer to SemVer for versioning
	cv::Mat cameraCapture, imageUndistorted, foundPath, convertedImage, convertedFullImage;//create all matrixes
    std::vector<cv::Mat> calibrationData =  cameraCalibration("./left%d.png"); //calculates radial distortion which can then be eliminated by calling cv::undistort(<input>, <output>, K, D);
	cv::Mat K = calibrationData[0]; //get the calibration data
	cv::Mat D = calibrationData[1]; //get the calibration data
	
	cv::Point firstFoundPoint;
	cv::Point lastFoundPoint;

	//start video capture
	//cv::VideoCapture input(0, cv::CAP_V4L2);
    //cv::VideoCapture set(cv::CAP_PROP_FPS, 120);
    cv::VideoCapture input("/home/jetson/Desktop/TestImages/testImage%d.png");
	
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierachy;
	std::vector<cv::Point> centers;
	std::vector<cv::Point> bestBranchPoints;
	int numBestPaths = 20; // Number of best paths to draw
	int bestBranchIndex;
	
	int fps;

	int rSpeed = 100;
	int lSpeed = 100;

	// Setup Serial Communication
	int fid= -1;
    uart_setup(fid);
    
    if (fid == -1) {
        std::cerr << "Error - Unable to open UART.  Ensure it is not in use by another application\n";
    }
    usleep(500000);   // 0.5 sec delay
	
	for(;;){

		rSpeed = 255;
		lSpeed = 255;

		if(!input.read(cameraCapture)){
			//throw std::invalid_argument("no camera image available"); //TODO: write some functioning error handling
			break;
		}
		centers.clear();

		
		cv::undistort(cameraCapture, imageUndistorted, K, D); //eliminate radial distortion

		convertedImage = convertImage(imageUndistorted, cv::Range(imageUndistorted.size().height - 50, imageUndistorted.size().height), cv::Range(0, imageUndistorted.size().width));
		convertedFullImage = convertImage(imageUndistorted, cv::Range(0, imageUndistorted.size().height), cv::Range(0, imageUndistorted.size().width));
		cv::findContours(convertedImage, contours, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0,imageUndistorted.size().height - 50)); //find contours in the mask
		
		centers = findContourCenters(contours, imageUndistorted);

		drawScanFlower(imageUndistorted, centers);
		
		cv::drawContours(imageUndistorted, contours, -1, cv::Scalar(0, 0, 255), 2); //draw the contours to visualize the found line
				
		for (int i = 0; i < centers.size(); i++) { // iterate over the amount of center pixels
			std::vector<std::vector<cv::Point>> branches = getBranchedPoints({centers[i]}, searchRadius, convertedFullImage, pathDepth);
			if (branches.empty()) {
				continue;
			}
			std::vector<double> branchGrade = checkBranchStraightnessAndLength(branches);

			// Sort branches by grade in descending order
			std::vector<int> indices(branchGrade.size());
			for (int j = 0; j < indices.size(); ++j) {
				indices[j] = j;
			}
			std::sort(indices.begin(), indices.end(), [&branchGrade](int a, int b) { return branchGrade[a] > branchGrade[b]; });
			
			bestBranchIndex = indices[0];  // First element after sorting is the best one
			bestBranchPoints = branches[bestBranchIndex];

			lastFoundPoint = bestBranchPoints.back();
			firstFoundPoint = centers[i];
			// Draw the top N best branches
			for (int j = 0; j < std::min(numBestPaths, static_cast<int>(indices.size())); j++) {
				int idx = indices[j];
				int colorIntensity =(j * (255 / numBestPaths)); // Diminishing green tones
				drawBranchWithGrade(imageUndistorted, branches[idx], branchGrade[idx], colorIntensity);
			}
		}
		// Find the angle between start and end in relation to a vertical line
		float angle = std::atan2(firstFoundPoint.y - lastFoundPoint.y, firstFoundPoint.x - lastFoundPoint.x) / M_PI;
		/*// between 0 and 0.5 go left, between 0.5 and 1 go right
		if (angle >= 0 && angle <= 0.5 - angleThresh) {
			
			lSpeed = -1*(255-angle*255);
			rSpeed = 255-angle*255;

			//std::string writeString = "R2:" + std::to_string(255-angle*255) + "\nL-1:" + std::to_string(255-angle*255) + "\n";
			//std::cout << write_serial(fid, writeString) << angle << std::endl; // turn left
		} else if (angle >= 0.5 + angleThresh && angle <= 1) {
			
			lSpeed = 255-angle*255;
			rSpeed = -1*(255-angle*255);

			//std::string writeString = "R-1:" + std::to_string((angle-1)*255) + "\nL2:" + std::to_string((angle-1)*255) + "\n";
			//std::cout << write_serial(fid, writeString) << angle << std::endl; // turn right
		} else if (angle > 0 - angleThresh && angle < 0 + angleThresh) {

			lSpeed = 255;
			rSpeed = 255;

			//std::string writeString = "R2:" + std::to_string(255) + "\nL2:" + std::to_string(255) + "\n";
			//std::cout << write_serial(fid, writeString) << angle << std::endl; // go forward
		} else {
			std::cout << "angle: " << angle << std::endl;
		}
		*/
		//relativePosition = (static_cast<double>(firstFoundPoint.x) / convertedImage.size().width) - 0.5;
		//relativePosition = calculateDistance(firstFoundPoint, cv::Point(imageUndistorted.size().height - 25, imageUndistorted.size().width / 2));
		
		if(firstFoundPoint.x < imageUndistorted.size().width / 2){
			relativePosition = imageUndistorted.size().width / 2 - firstFoundPoint.x;
		} else if(firstFoundPoint.x > imageUndistorted.size().width / 2){
			relativePosition = firstFoundPoint.x - imageUndistorted.size().width / 2;
		} else {
			relativePosition = 0;
		}

		relativePosition = relativePosition / (imageUndistorted.size().width / 2);

		cv::circle(imageUndistorted, cv::Point(imageUndistorted.size().width / 2, imageUndistorted.size().height - 25), 5, cv::Scalar(0, 0, 255), -1);
		
		//relativePosition = relativePosition / (imageUndistorted.size().width / 2);
		if (relativePosition > 0)  {
			//Left motor should be faster to steer right

			lSpeed = relativePosition * lSpeed + ((1 - angle)*255);
			rSpeed = (1 - relativePosition) * rSpeed + (1/3)*((1 - angle)*255);

		} else if (relativePosition < 0)  {
			//Right motor should be faster to steer left
			
			lSpeed = (1 - relativePosition) * lSpeed + (1/3)*((1 - angle) * 255);
			rSpeed = relativePosition * rSpeed + (1/3)*((1 - angle) * 255);
		}

		if(lSpeed > 255){
			lSpeed = 255;
		} else if(lSpeed < -255){
			lSpeed = -255;
		}
		if(rSpeed > 255){
			rSpeed = 255;
		} else if(rSpeed < -255){
			rSpeed = -255;
		}


		sendMotorData(rSpeed, lSpeed, fid);
		std::cout << "rSpeed: " << rSpeed << " lSpeed: " << lSpeed << " relative Position: " << relativePosition << std::endl;


		if (!convertedImage.empty()) {
			cv::imshow("Converted Image", convertedImage);
		} else {
			std::cerr << "Error: convertedImage is empty" << std::endl;
		}
		if (!convertedFullImage.empty()) {
			cv::imshow("Converted Full Image", convertedFullImage);
		} else {
			std::cerr << "Error: convertedFullImage is empty" << std::endl;
		}
		if (!imageUndistorted.empty()) {
			cv::imshow("Image Undistorted", imageUndistorted);
		} else {
			std::cerr << "Error: imageUndistorted is empty" << std::endl;
		}

		fps = drawFPS(imageUndistorted);

		cv::waitKey(); //continue when a key is pressed
			
		//exit when esc key is pressed
		char c = cv::waitKey(1);
		if(c == 27){
			std::string writeString = "R1:" + std::to_string(1) + "\nL1:" + std::to_string(1) + "\n";
			//std::cout << write_serial(fid, writeString) << angle << std::endl; // stop
			close(fid);
			break;
		} //27 = ESC key
	}

}
