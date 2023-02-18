#include <iostream>
#include <cmath>
#include <string>
#include <algorithm>
#include <vector>
#include <queue>
#include <stack>
#include <set>
#include <map>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>


#define M_PI                3.141592    // Pi
#define TO_DEG              (180/M_PI)  // Multiply by this constant to convert radians to degrees
#define ROVER_WIDTH         600         // Width of rover in mm
#define MAX_PITCH           25          // Maximum rover pitch angle in degrees
#define MED_PITCH           15          // Medium pitch angle threshold in degrees
#define MAX_ROLL            35          // Maximum rover roll angle in degrees
#define MED_ROLL            25          // Medium roll angle threshold in degrees

#define Y_MAX_ERROR         100         // Maximum y distance (in mm) between pixel and point on the ground, for pixel to not be outlier

#define BLACK               cv::Scalar(0, 0, 0)         // The color black in OpenCV (in BGR)
#define GRAY                cv::Scalar(125, 125, 125)   // The color gray in OpenCV (in BGR)
#define RED                 cv::Scalar(17, 31, 187)     // The color red in OpenCV (in BGR)
#define YELLOW              cv::Scalar(0, 181, 247)     // The color yellow in OpenCV (in BGR)




int main() {

    // Create black image
    cv::Mat img(120, 160, CV_8UC3, BLACK);


    cv::imshow("Image", img);
    cv::waitKey(0);
    cv::imwrite("C:/Users/Dew Bhaumik/Desktop/obstacle-avoidance/output.jpg", img);
    
    return 0;
}
