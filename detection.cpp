#include <iostream>
#include <fstream>
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

#define BLACK               cv::Vec3b(0, 0, 0)         // The color black in OpenCV (in BGR)
#define GRAY                cv::Vec3b(125, 125, 125)   // The color gray in OpenCV (in BGR)
#define RED                 cv::Vec3b(17, 31, 187)     // The color red in OpenCV (in BGR)
#define YELLOW              cv::Vec3b(0, 181, 247)     // The color yellow in OpenCV (in BGR)





class Pixel
{
    public:
        double x; // x value of pixel
        double y; // y value of pixel
        double z; // z value of pixel
};



/** 
 * Plugs in the z value of a point on a perfectly flat ground plane into a line equation representing this flat ground from the LiDAR's POV
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



/** 
 * Reads in XYZ values of each pixel in point cloud frame from CSV files before performing linear regression on this data
 * to estimate ground plane and output the slope and y-intercept of the line of best fit representing the straight line path
 * of the rover on the ground plane (please refer to documentation for detailed explanation)
 * 
 * @param x_filename The path to the CSV file containing the x values
 * @param y_filename The path to the CSV file containing the y values
 * @param z_filename The path to the CSV file containing the z values
 * @param pixels 2D vector containing Pixel objects that represent each pixel in 160 by 120 pixel point cloud frame
 * @param slope The outputted slope of the line of best
 * @param y_intercept The outputted y-intercept of the line of best fit 
 */ 
void linearRegression(const std::string &x_filename, const std::string &y_filename, const std::string &z_filename, std::vector<std::vector<Pixel>> &pixels, double &slope, double &y_intercept)
{
    std::ifstream x_file(x_filename);
    std::ifstream y_file(y_filename);
    std::ifstream z_file(z_filename);

    std::string line;

    int lines_to_skip = 17;

    // Skip over lines we don't want to read
    for(int i = 0; i < lines_to_skip; i++)
    {
        std::getline(x_file, line);
        std::getline(y_file, line);     
        std::getline(z_file, line);                 
    }

    char skip;

    for(int row = 0; row < pixels.size(); row++)
    {
        int characters_skipped = 0;
        int semicolon_count = 0;
        while(semicolon_count < 2)
        {
            z_file >> skip;

            characters_skipped++;
            if(skip == ';')
            {
                semicolon_count++;
            } 
            
        }
        for(int col = 0; col < pixels[row].size(); col++)
        {
            char character = ' ';
            std::string str;
            while(character != ';')
            {
                z_file >> character;
                str.push_back(character);
            }
            pixels[row][col].z = std::stod(str, NULL);
        }
        
        for(int i = 0; i < characters_skipped; i++)
        {
            z_file >> skip;
        }
    }
}



double GroundPlane(double z)
{
    return 0;
}



int main() {

    // Create black image
    cv::Mat img(120, 160, CV_8UC3, BLACK);

    
    // Initialize 2D vector of Pixel objects
    std::vector<std::vector<Pixel>> pixels(120, std::vector<Pixel>(160));

   
   // Initialize variables to store slope and y-intercept outputs of linearRegression() function
    double slope, y_intercept;
    
    linearRegression("x.csv", "y.csv", "z.csv", pixels, slope, y_intercept);

    
    // Iterate through pixels in original black image, coloring them as appropriate
    int row = 0;
    for(std::vector<Pixel> pixel_row : pixels)
    {
        int col = 0;
        for(Pixel pixel : pixel_row)
        {
            if(pixel.z != -1)
            {
                img.at<cv::Vec3b>(row, col) = RED;
            }
            col++;
        }
        row++;
    }


    // Display final image until user presses any button on the keyboard
    cv::imshow("Image", img);
    cv::waitKey(0);
    
    // Output final image to JPG file
    cv::imwrite("C:/Users/Dew Bhaumik/Desktop/obstacle-avoidance/output.jpg", img);
    
    return 0;
}
