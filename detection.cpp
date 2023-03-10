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
#define HEIGHT              730.0         // LiDAR's height above the ground in mm (distance from ground to LiDAR lens)
#define MAX_PITCH           25.0          // Maximum rover pitch angle in degrees
#define MAX_ROLL            35.0          // Maximum rover roll angle in degrees
#define GROUND              100.0         // Maximum absolute value y distance (in mm) from ground to pixel after transform() function has been called on pixel, for pixel to be considered ground 
#define SAFE                150.0         // Maximum absolute value y distance (in mm) for pixel to be considered safe/traversible without worry (e.g. not-worrisome rock/obstacle)

#define LIDAR_ANGLE         0.0                        // LiDAR's angle of inclination (relative to horizon) in degrees, where negative means tilted downwards and positive means tilted upwards
#define OMEGA               1.4521                     // Ground plane's angle of inclination (in degrees) as it appears in LiDAR image when LiDAR is upright (has 0 angle of inclination). Refer to documentation for more info
#define THETA               (OMEGA + LIDAR_ANGLE)      // Angle in degrees by which ground reference frame must be rotated to be parallel with LiDAR reference frame (refer to documentation to understand how THETA was determined)
#define PHI                 (90 - THETA)               // Please refer to documentation for explanation

#define BLACK               cv::Vec3b(0, 0, 0)         // The color black in OpenCV (in BGR)
#define GRAY                cv::Vec3b(125, 125, 125)   // The color gray in OpenCV (in BGR)
#define RED                 cv::Vec3b(17, 31, 187)     // The color red in OpenCV (in BGR)
#define YELLOW              cv::Vec3b(0, 181, 247)     // The color yellow in OpenCV (in BGR)
#define GREEN               cv::Vec3b(49, 164, 50)     // The color green in OpenCV (in BGR)





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
 * @param x_filename The absolute path to the CSV file containing the x values
 * @param y_filename The absolute path to the CSV file containing the y values
 * @param z_filename The absolute path to the CSV file containing the z values
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
            // PLEASE NOTE: We multiply the y value by -1 after reading it in because from the LiDAR's point-of-view, the positive y axis points downwards instead of upwards, so we flip the y axis and make "upwards" the positive direction
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
 * Transforms pixel from ground reference frame to LiDAR reference frame
 * 
 * @param angle_degrees Angle in degrees that pixel will be rotated by
 * @param y Y value of pixel
 * @param z Z value of pixel 
 */ 
void transform(const double &angle_degrees, double& y, double& z)
{
    double angle_radians = angle_degrees * TO_RAD; // Convert to radians
    
    z += HEIGHT * cos(PHI * TO_RAD);  // Translate pixel in z direction (please refer to documentation for detailed explanation)
    y += HEIGHT * sin(PHI * TO_RAD);  // Translate pixel in y direction (please refer to documentation for detailed explanation)

    // Rotate pixel about the x axis
    double cos_theta = cos(angle_radians);
    double sin_theta = sin(angle_radians);
    double y_new = y * cos_theta + z * sin_theta;
    double z_new = z * cos_theta - y * sin_theta;
    
    y = y_new;
    z = z_new;
}






/** 
 * Reads in XYZ values of each pixel in point cloud frame from CSV files into a 2D vector of Pixel objects
 * 
 * @param x_filename The absolute path to the CSV file containing the x values
 * @param y_filename The absolute path to the CSV file containing the y values
 * @param z_filename The absolute path to the CSV file containing the z values
 * @param pixels 2D vector containing Pixel objects that represent each pixel in 160 by 120 pixel point cloud frame
 */ 
void readInData(const std::string &x_filename, const std::string &y_filename, const std::string &z_filename, std::vector<std::vector<Pixel>> &pixels)
{
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
            // PLEASE NOTE: We multiply the y value by -1 after reading it in because from the LiDAR's point-of-view, the positive y axis points downwards instead of upwards, so we flip the y axis and make "upwards" the positive direction  
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
        }
        
        // Skip over last three characters of each row of each file
        for(int i = 0; i < characters_skipped; i++)
        {
            x_file >> skip;
            y_file >> skip;
            z_file >> skip;
        }
    }
}





int main() {

    // Create black image
    cv::Mat img(120, 160, CV_8UC3, BLACK);

    
    // Initialize 2D vector of Pixel objects
    std::vector<std::vector<Pixel>> pixels(120, std::vector<Pixel>(160));
    
    
    // Read in XYZ data
    readInData("absolute-path-to-x-values-file.csv", "absolute-path-to-y-values-file.csv", "absolute-path-to-z-values-file.csv", pixels);


    double x_dist;
    double y_dist;
    double z_dist;
    double pitch_angle;
    double roll_angle;
    

    // Iterate through pixels in original black image, coloring them as appropriate
    int row = 0;
    for(std::vector<Pixel> pixel_row : pixels)
    {
        int col = 0;
        for(Pixel pixel : pixel_row)
        {
            if(pixel.z != -1)  // Checks if pixel has valid depth value
            {   
                // Transform pixel's coordinates from ground reference frame to LiDAR reference frame
                transform(THETA, pixel.y, pixel.z);
                
                if(abs(pixel.y) <= GROUND)
                {
                    img.at<cv::Vec3b>(row, col) = GRAY; // Color the pixel gray
                    col++;
                    continue;
                }
                
                else if(abs(pixel.y) <= SAFE)
                {
                    img.at<cv::Vec3b>(row, col) = GREEN; // Color the pixel green
                    col++;
                    continue;
                }

                // Initialize ground_pixel object 
                Pixel ground_pixel;
                ground_pixel.x = 0;
                ground_pixel.y = 0;
                ground_pixel.z = 0;
                
                // Find the closest pixel on the ground (that has a valid depth value) that is in the same column (of the LiDAR image) as the original pixel, and store its XYZ values in the ground_pixel object
                int next_row = row + 1;
                while(next_row < 120 && pixels[next_row][col].z != -1)  // Keep searching for this ground pixel until last possible row has been iterated through or a pixel of invalid depth is found  
                {
                    ground_pixel = pixels[next_row][col];
                    transform(THETA, ground_pixel.y, ground_pixel.z);
                    if(abs(ground_pixel.y) <= GROUND)
                    {
                        break;
                    }
                    next_row++;
                }


                x_dist = abs(pixel.x) + ROVER_WIDTH / 2;  // X distance from pixel to the farthest point on the front wheel that the rover will pivot on when rolling
                y_dist = abs(pixel.y - ground_pixel.y);  // Y distance from pixel to ground_pixel (the closest pixel on the ground that's in the same column)
                z_dist = abs(pixel.z - ground_pixel.z);  // Z distance from pixel to ground_pixel (the closest pixel on the ground that's in the same column)

                // Determine pitch and roll angles
                pitch_angle = atan2(y_dist, z_dist) * TO_DEG;
                roll_angle = atan2(y_dist, x_dist) * TO_DEG;

                
                if(pitch_angle > MAX_PITCH || roll_angle > MAX_ROLL)
                {
                    img.at<cv::Vec3b>(row, col) = RED; // Color the pixel red
                }
                else
                {
                    img.at<cv::Vec3b>(row, col) = YELLOW; // Color the pixel yellow
                }
            }
            col++;
        }
        row++;
    }

    
    // Output final image to JPG file
    cv::imwrite("C:/Users/Dew Bhaumik/Desktop/obstacle-avoidance/output.jpg", img);
    
    return 0;
}
