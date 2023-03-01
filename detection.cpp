#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <algorithm>
#include <vector>


#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>



#define M_PI                3.141592      // Pi
#define TO_DEG              (180/M_PI)    // Multiply by this constant to convert radians to degrees
#define TO_RAD              (M_PI/180)    // Multiply by this constant to convert degrees to radians
#define ROVER_WIDTH         680.0         // Width of rover in mm
#define HEIGHT              635.0         // LiDAR's height above the ground in mm (distance from ground to LiDAR lens)
#define MAX_PITCH           25.0          // Maximum rover pitch angle in degrees
#define MED_PITCH           15.0          // Medium pitch angle threshold in degrees
#define MAX_ROLL            35.0          // Maximum rover roll angle in degrees
#define MED_ROLL            25.0          // Medium roll angle threshold in degrees

#define THETA               -0.694997     // The slope of a perfectly flat ground plane from the LiDAR's POV

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
 * Reads in XYZ values of each pixel in point cloud frame from CSV files before performing linear regression on this data
 * to estimate ground plane and output the slope and y-intercept of a line on the ground plane (please refer to documentation for detailed explanation)
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
    int total_num_points = 0;
    double y_total = 0;
    double z_total = 0;
    double zy_total = 0;
    double z_squared_total = 0;

    // Read in CSV files
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

    // Iterate through each row in CSV files
    for(int row = 0; row < pixels.size(); row++)
    {
        int characters_skipped = 0;
        int semicolon_count = 0;
        
        // Skip over first three characters of each row of each file
        while(semicolon_count < 2)
        {
            x_file >> skip;
            y_file >> skip;
            z_file >> skip;
            characters_skipped++;
            
            if(skip == ';')
            {
                semicolon_count++;
            } 
            
        }

        // Iterate through each column in CSV files
        for(int col = 0; col < pixels[row].size(); col++)
        {
            char character_x = ' ';
            char character_y = ' ';
            char character_z = ' ';
            
            std::string str_x;
            std::string str_y;
            std::string str_z;
            
            // Read in x values while skipping semicolons 
            while(character_x != ';')
            {
                x_file >> character_x;
                str_x.push_back(character_x);
            }
            pixels[row][col].x = std::stod(str_x, NULL);  // Convert x value from string to double

            // Read in y values while skipping semicolons
            while(character_y != ';')
            {
                y_file >> character_y;
                str_y.push_back(character_y);
            }
            pixels[row][col].y = std::stod(str_y, NULL) * -1;  // Convert y value from string to double

            // Read in z values while skipping semicolons
            while(character_z != ';')
            {
                z_file >> character_z;
                str_z.push_back(character_z);
            }
            pixels[row][col].z = std::stod(str_z, NULL);  // Convert z value from string to double


            // Consider pixel for linear regression calculation if pixel has valid depth value
            if(pixels[row][col].z != -1)
            {
                double pixel_z = pixels[row][col].z;
                double pixel_y = pixels[row][col].y;
        
                total_num_points++;
                y_total += pixel_y;
                z_total += pixel_z;
                zy_total += pixel_z * pixel_y;
                z_squared_total += pixel_z * pixel_z;
             }
        }
        
        // Skip over last three characters of each row of each file
        for(int i = 0; i < characters_skipped; i++)
        {
            x_file >> skip;
            y_file >> skip;
            z_file >> skip;
        }
    }

    // Calculate slope and y-intercept using method of least squares
    slope = (total_num_points * zy_total - z_total * y_total) / (total_num_points * z_squared_total - z_total * z_total);
    double y_mean = y_total / total_num_points;
    double z_mean = z_total / total_num_points;
    y_intercept = y_mean - slope * z_mean;
}





/** 
 * Rotates a pixel about the x-axis before translating it downwards by the vertical height of the LiDAR lens from the ground
 * 
 * @param angle_degrees Angle in degrees that pixel will be rotated by
 * @param y Y value of pixel
 * @param z Z value of pixel 
 */ 
void transform(const double &angle_degrees, double& y, double& z)
{
    double angle_radians = angle_degrees * TO_RAD; // convert to radians
    
    // Rotate pixel about the x axis
    double cos_theta = cos(angle_radians);
    double sin_theta = sin(angle_radians);
    double y_new = y * cos_theta + z * sin_theta;
    double z_new = z * cos_theta - y * sin_theta;
    
    // Vertically translate the pixel downwards by the height of the LiDAR from the ground
    y_new -= HEIGHT;

    y = y_new;
    z = z_new;
}





int main() {

    // Create black image
    cv::Mat img(120, 160, CV_8UC3, BLACK);

    
    // Initialize 2D vector of Pixel objects
    std::vector<std::vector<Pixel>> pixels(120, std::vector<Pixel>(160));

   
   // Initialize variables to store slope and y-intercept outputs of linearRegression() function
    double slope, y_intercept;
    
    
    // // Iterate through pixels in original black image, coloring them as appropriate
    // int row = 0;
    // for(std::vector<Pixel> pixel_row : pixels)
    // {
    //     int col = 0;
    //     for(Pixel pixel : pixel_row)
    //     {
    //         if(pixel.z != -1)
    //         {
    //             img.at<cv::Vec3b>(row, col) = RED;
    //         }
    //         col++;
    //     }
    //     row++;
    // }

    
    // Output final image to JPG file
    cv::imwrite("C:/Users/Dew Bhaumik/Desktop/obstacle-avoidance/output.jpg", img);
    
    return 0;
}
