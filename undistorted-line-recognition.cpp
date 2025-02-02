#define _USE_MATH_DEFINES
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>

// Constants
const int WHITE_BLACK_THRESHOLD = 50;
const int PATH_DEPTH = 2;
const int ERODE_ITERATIONS = 3;
const int DILATE_ITERATIONS = 4;
const int SEARCH_RADIUS = 120;
const int SEARCH_POINTS = 15;
const std::string VERSION = "0.1.1";

// Function to find surrounding pixels
std::vector<cv::Point> findSurroundingPixels(const cv::Mat& image, const cv::Point& centerPixel, int radius, int pointAmount) {
    std::vector<cv::Point> points;
    for (int i = 0; i < pointAmount; ++i) {
        double angle = 2.0 * M_PI * i / pointAmount;
        cv::Point point(centerPixel.x + radius * sin(angle), centerPixel.y - radius * cos(angle));
        if (point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows) {
            points.push_back(point);
        }
    }
    return points;
}

// Function to draw scan flower
void drawScanFlower(cv::Mat& image, const std::vector<cv::Point>& centers) {
    for (const auto& center : centers) {
        std::vector<cv::Point> nextPixels = findSurroundingPixels(image, center, SEARCH_RADIUS, SEARCH_POINTS);
        for (const auto& point : nextPixels) {
            cv::line(image, center, point, cv::Scalar(255, 255, 255), 1);
            cv::circle(image, point, 2, cv::Scalar(255, 0, 0), -1);
        }
    }
}

// Function for camera calibration
std::vector<cv::Mat> cameraCalibration(const std::string& path) {
    cv::Mat image, image_gray, K, D;
    std::vector<cv::Mat> Rs, Ts;
    cv::VideoCapture calib(path);
    if (!calib.isOpened()) {
        throw std::runtime_error("Error: Unable to open calibration video file.");
    }
    std::vector<cv::Point2f> corners_image;
    std::vector<std::vector<cv::Point2f>> points_image;
    cv::Size pattern_size(8, 6);
    std::vector<cv::Mat> calibrationData;

    while (calib.read(image)) {
        cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
        bool found = cv::findChessboardCorners(image_gray, pattern_size, corners_image);
        if (found) {
            cv::drawChessboardCorners(image, pattern_size, corners_image, found);
            points_image.push_back(corners_image);
        }
    }

    if (points_image.empty()) {
        throw std::runtime_error("Error: No chessboard corners found in calibration images.");
    }

    std::vector<cv::Point3f> corners_world;
    for (int i = 0; i < pattern_size.height; ++i) {
        for (int j = 0; j < pattern_size.width; ++j) {
            corners_world.push_back(cv::Point3f(j, i, 0));
        }
    }

    std::vector<std::vector<cv::Point3f>> points_world(points_image.size(), corners_world);
    double rms = cv::calibrateCamera(points_world, points_image, image_gray.size(), K, D, Rs, Ts);
    std::cout << "Reprojection error: " << rms << std::endl;

    calibrationData.push_back(K);
    calibrationData.push_back(D);
    return calibrationData;
}

// Function to find path
cv::Mat findPath(int repetitions, int searchRadius, const cv::Point& startpoint, const cv::Mat& image, cv::Mat& outputImage) {
    std::vector<cv::Point> nextPixels = findSurroundingPixels(image, startpoint, searchRadius, SEARCH_POINTS);
    std::vector<cv::Point> validPixels, goingToCheck = {startpoint};

    for (const auto& point : nextPixels) {
        if (point.x > 0 && point.x < image.cols - 1 && point.y > 0 && point.y < image.rows - 1 && image.at<uchar>(point) == 255) {
            cv::line(outputImage, startpoint, point, cv::Scalar(0, 255, 0), 1);
            goingToCheck.push_back(point);
        }
    }

    for (int i = 0; i < repetitions; ++i) {
        for (const auto& checkPoint : goingToCheck) {
            nextPixels = findSurroundingPixels(image, checkPoint, searchRadius, SEARCH_POINTS);
            for (const auto& point : nextPixels) {
                if (point.x > 0 && point.x < image.cols - 1 && point.y > 0 && point.y < image.rows - 1 && image.at<uchar>(point) == 255) {
                    cv::line(outputImage, checkPoint, point, cv::Scalar(0, 255, 0), 1);
                    validPixels.push_back(point);
                }
            }
        }
        goingToCheck = validPixels;
        validPixels.clear();
    }
    return outputImage;
}

// Function to convert image
cv::Mat convertImage(cv::Mat image, const cv::Range& cropHeight, const cv::Range& cropWidth) {
    cv::Mat image_gray, image_thresholded, image_thresholded_eroded, image_thresholded_dilated;

    image = image(cropHeight, cropWidth);
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
    cv::threshold(image_gray, image_thresholded, WHITE_BLACK_THRESHOLD, 255, cv::THRESH_BINARY_INV);
    cv::erode(image_thresholded, image_thresholded_eroded, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10)), cv::Point(-1, -1), ERODE_ITERATIONS);
    cv::dilate(image_thresholded_eroded, image_thresholded_dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10)), cv::Point(-1, -1), DILATE_ITERATIONS);
    return image_thresholded_dilated;
}

// Function to process frame
void processFrame(const cv::Mat& cameraCapture, const cv::Mat& K, const cv::Mat& D) {
    cv::Mat imageUndistorted, convertedImage, foundPath;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point> centers;

    cv::undistort(cameraCapture, imageUndistorted, K, D);
    convertedImage = convertImage(imageUndistorted, cv::Range(imageUndistorted.rows - 50, imageUndistorted.rows), cv::Range(0, imageUndistorted.cols));
    cv::findContours(convertedImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, imageUndistorted.rows - 50));

    for (const auto& contour : contours) {
        cv::Moments M = cv::moments(contour);
        if (M.m00 != 0) {
            cv::Point2f center(M.m10 / M.m00, M.m01 / M.m00);
            centers.push_back(center);
            cv::circle(imageUndistorted, center, 5, cv::Scalar(100, 0, 250), -1);
        }
    }

    drawScanFlower(imageUndistorted, centers);
    cv::drawContours(imageUndistorted, contours, -1, cv::Scalar(0, 0, 255), 2);

    for (const auto& center : centers) {
        foundPath = findPath(PATH_DEPTH, SEARCH_RADIUS, center, convertImage(imageUndistorted, cv::Range(0, imageUndistorted.rows), cv::Range(0, imageUndistorted.cols)), imageUndistorted);
        if (foundPath.empty()) {
            foundPath = imageUndistorted;
        }
    }

    if (!imageUndistorted.empty()) {
        cv::imshow("line full", imageUndistorted);
    } else {
        std::cerr << "Error: imageUndistorted is empty" << std::endl;
    }
    if (!convertedImage.empty()) {
        cv::imshow("line", convertedImage);
    } else {
        std::cerr << "Error: convertedImage is empty" << std::endl;
    }
}

int main() {
    try {
        std::cout << "Started line recognition. Running: " << VERSION << std::endl;
        std::vector<cv::Mat> calibrationData = cameraCalibration("./left%d.png");
        cv::Mat K = calibrationData[0];
        cv::Mat D = calibrationData[1];

        cv::VideoCapture input("/home/jetson/Desktop/TestImages/testImage%d.png");
        if (!input.isOpened()) {
            throw std::runtime_error("Error: Unable to open video file.");
        }

        while (true) {
            cv::Mat cameraCapture;
            if (!input.read(cameraCapture)) {
                std::cerr << "Error: Unable to read frame from video file." << std::endl;
                break;
            }

            processFrame(cameraCapture, K, D);

            if (cv::waitKey(1) == 27) { // 27 = ESC key
                break;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return -1;
    }
    return 0;
}
