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


#define M_PI                3.141592      // Pi
#define TO_DEG              (180/M_PI)    // Multiply by this constant to convert radians to degrees
#define ROVER_WIDTH         650.0         // Width of rover in mm
#define HEIGHT              575.0         // LiDAR's height above the ground in mm (distance from ground to bottom of LiDAR)
#define MAX_PITCH           25.0          // Maximum rover pitch angle in degrees
#define MED_PITCH           15.0          // Medium pitch angle threshold in degrees
#define MAX_ROLL            35.0          // Maximum rover roll angle in degrees
#define MED_ROLL            25.0          // Medium roll angle threshold in degrees

#define GROUND_SLOPE        (75.0/1356)   // The slope of a perfectly flat ground plane from the LiDAR's POV
#define Y_MAX_ERROR         50            // Maximum y distance (in mm) between pixel and point on the ground, for pixel to not be outlier

#define BLACK               cv::Scalar(0, 0, 0)         // The color black in OpenCV (in BGR)
#define GRAY                cv::Scalar(125, 125, 125)   // The color gray in OpenCV (in BGR)
#define RED                 cv::Scalar(17, 31, 187)     // The color red in OpenCV (in BGR)
#define YELLOW              cv::Scalar(0, 181, 247)     // The color yellow in OpenCV (in BGR)






/** 
 * PLugs in the z value of a point on a perfectly flat ground plane into a line equation representing this flat ground from the LiDAR's POV
 * when mounted a distance HEIGHT above this ground, and returns the point's corresponding y value
 * 
 * @param z The z value of a point on a perfectly flat ground plane from LiDAR's POV
 * @return Corresponding y value of point
 */   
double GroundLine(double z)
{
    double y;
    y = GROUND_SLOPE * (z - 231.0) - 25.0 - HEIGHT;  /* Slope-point form line equation where 231.0 is the z1 value and 
                                                        25.0 is the y1 value (please refer to documentation for detailed explanation) */                                                
    return y;
}


class Pixel
{
    public:
        double x; // x value of pixel
        double y; // y value of pixel
        double z; // z value of pixel
        cv::Scalar color; // color of pixel
};


int main() {

    // Create black image
    cv::Mat img(120, 160, CV_8UC3, BLACK);

    // Display final image until user presses any button on the keyboard
    cv::imshow("Image", img);
    cv::waitKey(0);
    
    // Output final image to JPG file
    cv::imwrite("C:/Users/Dew Bhaumik/Desktop/obstacle-avoidance/output.jpg", img);
    
    return 0;
}
