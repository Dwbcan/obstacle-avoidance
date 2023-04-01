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
#define ROVER_LENGTH        740.0         // Length of rover in mm
#define WHEEL_RAD           150.0         // Wheel radius in mm
#define HEIGHT              730.0         // LiDAR's height above the ground in mm (distance from bottom of rover to LiDAR lens)
#define MAX_PITCH           25.0          // Maximum rover pitch angle in degrees
#define MAX_ROLL            35.0          // Maximum rover roll angle in degrees
#define GROUND              100.0         // Maximum absolute value y distance (in mm) from ground to pixel, for pixel to be considered ground in obstacle detection algorithm
#define SAFE                150.0         // Maximum height above ground (in mm) for pixel to be considered safe/traversible without worry (e.g. not-worrisome rock/obstacle)

#define LIDAR_ANGLE         0.0           // LiDAR's angle of inclination (relative to horizon) in degrees, where negative means tilted downwards and positive means tilted upwards
#define OUTLIER             50.0          // Maximum absolute value height of pixel (in mm) from ground, to not be considered an outlier in ground plane estimation with linear regression
#define PIX_PER_DEG         (160.0/90)    // Since LiDAR has 90 degree horizontal FOV and LiDAR image has 160 pixel width, for every degree we move horizontally in 3D space, we move PIX_PER_DEG pixels horizontally in LiDAR image

#define BLACK               cv::Vec3b(0, 0, 0)         // The color black in OpenCV (in BGR)
#define GRAY                cv::Vec3b(125, 125, 125)   // The color gray in OpenCV (in BGR)
#define RED                 cv::Vec3b(17, 31, 187)     // The color red in OpenCV (in BGR)
#define YELLOW              cv::Vec3b(0, 181, 247)     // The color yellow in OpenCV (in BGR)
#define GREEN               cv::Vec3b(49, 164, 50)     // The color green in OpenCV (in BGR)
#define BLUE                cv::Vec3b(128, 0, 0)       // The color navy blue in OpenCV (in BGR)





class Pixel
{
    public:
        double x;  // X value of pixel
        double y;  // Y value of pixel
        double z;  // Z value of pixel
        std::string color;  // Color of pixel
};





/** 
 * Transforms pixel from ground reference frame to LiDAR reference frame
 * 
 * @param angle_degrees Angle in degrees that pixel will be rotated by about x axis to be parallel with LiDAR's reference frame
 * @param y Y value of pixel
 * @param z Z value of pixel 
 */ 
void transform(const double &angle_degrees, double& y, double& z)
{
    double angle_radians = angle_degrees * TO_RAD; // Convert to radians

    double phi = 90.0 - LIDAR_ANGLE;  // Angle in degrees used to determine translations in z and y directions that will translate pixel to LiDAR's reference frame
    
    z += HEIGHT * cos(phi * TO_RAD);  // Translate pixel in z direction (please refer to documentation for detailed explanation)
    y += HEIGHT * sin(phi * TO_RAD);  // Translate pixel in y direction (please refer to documentation for detailed explanation)

    // Rotate pixel about x axis
    double cos_theta = cos(angle_radians);
    double sin_theta = sin(angle_radians);
    double y_new = y * cos_theta + z * sin_theta;
    double z_new = z * cos_theta - y * sin_theta;
    
    y = y_new;
    z = z_new;
}





/** 
 * Reads in XYZ values of each pixel in LiDAR image from CSV files into a 2D vector of Pixel objects before performing linear regression on this data 
 * to estimate the ground plane and output its slope (please refer to documentation for detailed explanation)
 * 
 * @param x_filename The absolute path to the CSV file containing the x values (please see sample CSV files in repo)
 * @param y_filename The absolute path to the CSV file containing the y values (please see sample CSV files in repo)
 * @param z_filename The absolute path to the CSV file containing the z values (please see sample CSV files in repo)
 * @param pixels 2D vector containing Pixel objects that represent each pixel in 160 by 120 pixel LiDAR image
 * @param ground_slope Variable to store the ground plane's slope output in degrees
 * @param found_pixel_row Variable to store the row number of the first pixel with a valid depth value that was found (this info will be used later in median filtering)
 * @param found_pixel_col Variable to store the column number of the first pixel with a valid depth value that was found (this info will be used later in median filtering)
 */ 
void readInData(const std::string &x_filename, const std::string &y_filename, const std::string &z_filename, std::vector<std::vector<Pixel>> &pixels, double &ground_slope, int &found_pixel_row, int &found_pixel_col)
{
    // Initalize temporary Pixel object to check if current pixel should be considered for linear regression calculation
    Pixel pixel;

    bool found_valid_pixel = false;  // Flag variable used to indicate that the first pixel with a valid depth value has been found

    int total_num_points = 0;  // Total number of pixels used in linear regression calculation
    double y_total = 0;  // Sum of y values of all pixels used in linear regression calculation
    double z_total = 0;  // Sum of z values of all pixels used in linear regression calculation
    double zy_total = 0;  // Sum of all z values multiplied by their corresponding y values for all pixels used in linear regression calculation
    double z_squared_total = 0;  // Sum of squared z values for all pixels used in linear regression calculation

    // Read in CSV files
    std::ifstream x_file(x_filename);
    std::ifstream y_file(y_filename);
    std::ifstream z_file(z_filename);

    std::string line;

    int lines_to_skip = 17;  // Number of lines to skip in CSV file

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
        
        // Skip over first characters of each row of each file until two consecutive semicolons have been skipped (see sample CSV files in repo)
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
            
            // Read in z values while skipping semicolons
            while(character_z != ';')
            {
                z_file >> character_z;
                str_z.push_back(character_z);
            }
            pixels[row][col].z = std::stod(str_z, NULL);  // Convert z value from string to double

            // If z value is invalid (has a value of -1), skip the corresponding x and y values of the current pixel in the LiDAR image
            if(pixels[row][col].z == -1)
            {
                while(character_x != ';')
                {
                    x_file >> character_x;
                }
                while(character_y != ';')
                {
                    y_file >> character_y;
                }
                continue;
            }
            
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


            // If the current pixel is the first pixel in the LiDAR image with a valid depth value, store the row and column number of this pixel
            if(!found_valid_pixel)
            {
                found_valid_pixel = true;
                found_pixel_row = row;
                found_pixel_col = col;
            }

            pixel = pixels[row][col];

            transform(LIDAR_ANGLE, pixel.y, pixel.z);  // Transform pixel to LiDAR's reference frame
            
            // Consider pixel for linear regression calculation if pixel is within a rover length away from LiDAR and isn't an outlier
            if(abs(pixel.y) <= OUTLIER && pixel.z <= ROVER_LENGTH)
            {
                double pixel_z = pixel.z;
                double pixel_y = pixel.y;
        
                total_num_points++;
                y_total += pixel_y;
                z_total += pixel_z;
                zy_total += pixel_z * pixel_y;
                z_squared_total += pixel_z * pixel_z;
            }
        }

        // Skip over last characters of each row of each file starting from the first semicolon of two consecutive semicolons (see sample CSV files in repo)
        // PLEASE NOTE: We skip over the last "characters_skipped - 1" characters because in the most recent iteration of the for loop above, we have already read in the first semicolon of the two consecutive semicolons
        for(int i = 0; i < characters_skipped - 1; i++)
        {
            x_file >> skip;
            y_file >> skip;
            z_file >> skip;
        }
    }

    // Calculate slope using method of least squares
    if(total_num_points * z_squared_total - z_total * z_total != 0)  // Check to prevent dividing by zero
    {
        ground_slope = (total_num_points * zy_total - z_total * y_total) / (total_num_points * z_squared_total - z_total * z_total);
        ground_slope = atan(ground_slope) * TO_DEG;
    }
}





/** 
 * Finds and returns the median value of a vector of doubles
 * 
 * @param kernel The vector of doubles whose median value will be returned
 */ 
double findMedian(std::vector<double>& kernel) {
    
    // Find the middle index of the vector
    int middle = int(kernel.size() / 2);
    
    // Use nth_element algorithm to find the middle value
    std::nth_element(kernel.begin(), kernel.begin() + middle, kernel.end());
    
    // If the vector has an odd number of elements, the middle value is the median
    if (kernel.size() % 2 != 0) {
        return kernel[middle];
    }
    
    // If the vector has an even number of elements, the median is the average of the two middle values
    else {
        std::nth_element(kernel.begin(), kernel.begin() + middle - 1, kernel.end());
        return (kernel[middle - 1] + kernel[middle]) / 2.0;
    }
}





/** 
 * Calculates current pixel's pitch angle with reference to a nearby pixel in the same column of the LiDAR image
 * 
 * @param pixels 2D vector containing Pixel objects that represent each pixel in 160 by 120 pixel LiDAR image
 * @param row Current row in vector of Pixel objects
 * @param col Current column in vector of Pixel objects
 * @param curr_pixel Current Pixel object in vector of Pixel objects
 * @param pitch_ref_pixel Reference pixel (the current pixel's pitch angle with reference to this pixel will be determined)
 * @param theta Angle in degrees that pixel will be rotated by about x axis to be parallel with LiDAR's reference frame
 * @param pitch_angle Variable to store pitch angle output (in degrees) between curr_pixel and pitch_ref_pixel
 */ 
void calculatePitch(const std::vector<std::vector<Pixel>> &pixels, const int &row, const int &col, const Pixel &curr_pixel, Pixel &pitch_ref_pixel, const double &theta, double &pitch_angle)
{
    bool found_ref_pixel = false;  // Flag variable used to break while loop when a pixel with a valid depth value is found
    
    int curr_row = row + 3;

    // Starting from a row 3 rows below current pixel, search the image from bottom to top for a pixel (in the same column) with a valid depth value and set this pixel as the reference pixel
    while(curr_row > row)
    {
        // Check if pixel exists and has a valid depth value
        if(curr_row <= 119 && pixels[curr_row][col].z != -1)
        {
            pitch_ref_pixel = pixels[curr_row][col];
            transform(theta, pitch_ref_pixel.y, pitch_ref_pixel.z);  // Transform pixel from ground's reference frame to LiDAR's reference frame

            double y_dist = abs(curr_pixel.y - pitch_ref_pixel.y);  // Y distance from current pixel to reference pixel
            double z_dist = abs(curr_pixel.z - pitch_ref_pixel.z);  // Z distance from current pixel to reference pixel

            // Determine pitch angle of the current pixel with reference to the reference pixel
            pitch_angle = atan2(y_dist, z_dist) * TO_DEG;
            
            found_ref_pixel = true;
            break;
        }
        curr_row--;
    }

    // In the case that a reference pixel was not found, set pitch_angle to exceed MAX_PITCH threshold just to be safe
    if(!found_ref_pixel)
    {
        pitch_angle = MAX_PITCH + 1;
    }
}





/** 
 * Calculates roll angle between rover's front left and front right wheels 
 * 
 * @param pixels 2D vector containing Pixel objects that represent each pixel in 160 by 120 pixel LiDAR image
 * @param col Current column in vector of Pixel objects
 * @param curr_pixel Current Pixel object in vector of Pixel objects (we'll assume bottom centre of rover's front face will be located at curr_pixel's position in 3D space)
 * @param theta Angle in degrees that pixel will be rotated by about x axis to be parallel with LiDAR's reference frame
 * @param roll_angle Variable to store roll angle output (in degrees) between rover's front left and right wheels
 */ 
void calculateRoll(const std::vector<std::vector<Pixel>> &pixels, const int &col, const Pixel &curr_pixel, const double &theta, double &roll_angle)
{
    // Initialize left_pixel and right_pixel objects (curr_pixel represents bottom centre of rover's front face, so left_pixel and right_pixel represent rover's left and right wheels)
    Pixel left_pixel = curr_pixel;
    Pixel right_pixel = curr_pixel;

    // Set x coordinates (left_pixel is a half rover width to the left of curr_pixel while right_pixel is a half rover width to the right)
    left_pixel.x = curr_pixel.x - ROVER_WIDTH / 2;
    right_pixel.x = curr_pixel.x + ROVER_WIDTH / 2;

    double left_angle = abs(atan2(left_pixel.x, left_pixel.z) * TO_DEG - atan2(curr_pixel.x, curr_pixel.z) * TO_DEG);  // Yaw angle between left_pixel and curr_pixel (in degrees)
    double right_angle = abs(atan2(right_pixel.x, right_pixel.z) * TO_DEG - atan2(curr_pixel.x, curr_pixel.z) * TO_DEG);  // Yaw angle between right_pixel and curr_pixel (in degrees)
    
    int move_left = int(ceil(left_angle * PIX_PER_DEG));  // Amount of pixels to move left from curr_pixel to reach left_pixel
    int move_right = int(ceil(right_angle * PIX_PER_DEG));  // Amount of pixels to move right from curr_pixel to reach right_pixel

    int col_left = col - move_left;  // Column of left_pixel in LiDAR image
    int col_right = col + move_right;  // Column of right_pixel in LiDAR image

    // Ensure col_left is within bounds
    if(col_left < 0)
    {
        col_left = 0;
    }

    // Ensure col_right is within bounds
    if(col_right >= 160)
    {
        col_right = 159;
    }

    bool found_left_pixel = false;  // Flag variable used to indicate left_pixel has been found
    bool found_right_pixel = false;  // Flag variable used to indicate right_pixel has been found

    /* Starting from the left-most and right-most columns of the LiDAR image, iterate towards curr_pixel's column from both sides 
       until both left_pixel and right_pixel have been found */
    while(col_left < col && col_right > col)
    {
        int curr_row = 0;

        if(!found_left_pixel || !found_right_pixel)
        {
            /* Starting from the top of the column, search for left_pixel and right_pixel in their respective columns. 
               The left_pixel and right_pixel objects will be the first pixels from the top of their column that are within 100 mm of curr_pixel */
            while(curr_row < 120)
            {
                // If left_pixel hasn't already been found and pixel has a valid depth value within 100 mm of curr_pixel's depth value, set left_pixel to this pixel
                if(!found_left_pixel && pixels[curr_row][col_left].z != -1)
                {
                    left_pixel = pixels[curr_row][col_left];
                    transform(theta, left_pixel.y, left_pixel.z);  // Transform pixel from ground's reference frame to LiDAR's reference frame

                    if(abs(left_pixel.z - curr_pixel.z) <= 100)
                    {
                        found_left_pixel = true;
                    }
                }
                
                // If right_pixel hasn't already been found and pixel has a valid depth value within 100 mm of curr_pixel's depth value, set right_pixel to this pixel
                if(!found_right_pixel && pixels[curr_row][col_right].z != -1)
                {
                    right_pixel = pixels[curr_row][col_right];
                    transform(theta, right_pixel.y, right_pixel.z);  // Transform pixel from ground's reference frame to LiDAR's reference frame

                    if(abs(right_pixel.z - curr_pixel.z) <= 100)
                    {
                        found_right_pixel = true;
                    }
                }

                // If both left_pixel and right_pixel have been found, break inner while loop
                if(found_left_pixel && found_right_pixel)
                {
                    break;
                }
                curr_row++;
            }
        }

        // If left_pixel and right_pixel is found, calculate roll angle between them and break outer while loop
        if(found_left_pixel && found_right_pixel)
        {
            double x_dist = abs(left_pixel.x - right_pixel.x);  // X distance from left_pixel to right_pixel
            double y_dist = abs(left_pixel.y - right_pixel.y);  // Y distance from left_pixel to right_pixel

            // Determine roll angle between left_pixel and right_pixel (this is the roll angle between rover's left and right wheels if bottom centre of rover is located at curr_pixel's position in 3D space)
            roll_angle = atan2(y_dist, x_dist) * TO_DEG;
            break;
        }
        col_left++;
        col_right--;
    }
    
    // In the case that either left_pixel or right_pixel was not found, set roll_angle to exceed MAX_ROLL threshold just to be safe
    if(!found_left_pixel || !found_right_pixel)
    {
        roll_angle = MAX_ROLL + 1;
    }
}





/** 
 * Takes in image and colors all pixels that could represent ditches or gaps in depth value between two objects blue
 * 
 * @param img Image to perform ditch detection on 
 * @param pixels 2D vector containing Pixel objects that represent each pixel in 160 by 120 pixel LiDAR image
 * @param theta Angle in degrees that pixel will be rotated by about x axis to be parallel with LiDAR's reference frame
 */ 
void detectDitch(cv::Mat &img, std::vector<std::vector<Pixel>> &pixels, const double &theta)
{
    bool found_pixel = false;  // Flag variable used to break the while loops when a pixel with a valid depth value is found

    // Loop through every pixel in the 2D vector of Pixel objects until a pixel with a valid depth value is found
    int row = 0;
    while(row < 120)
    {
        int col = 0;
        while(col < 160)
        {
            // Break inner while loop if a pixel with a valid depth value is found
            if(pixels[row][col].z != -1)
            {
                found_pixel = true;
                break;
            }
            col++;
        }

        // Break outer while loop if a pixel with a valid depth value is found
        if(found_pixel)
        {
            break;
        }
        row++;
    }

    int found_pixel_row = row;  // Store the row number of the first pixel with a valid depth value that was found

    bool found_ground = false;  // Flag variable used to break the while loops when a pixel on the ground is found
    
    // Initialize temporary Pixel object which will be used to transform pixel from ground's reference frame to LiDAR's reference frame
    Pixel pixel;

    // Loop through every pixel in the rest of the 2D vector of Pixel objects until a pixel on the ground is found
    while(row < 120)
    {
        int col = 0;
        while(col < 160)
        {   
            if(pixels[row][col].z != -1)  // Check pixel's depth value so we can skip pixels with invalid depth values
            {
                pixel = pixels[row][col];
                transform(theta, pixel.y, pixel.z);  // Transform the pixel from the ground's reference frame to the LiDAR's reference frame if it has a valid depth value
                
                // Break inner while loop if a pixel on the ground is found
                if(abs(pixel.y) <= GROUND)
                {
                    found_ground = true;
                    break;
                }
            }
            col++;
        }

        // Break outer while loop if a pixel on the ground is found
        if(found_ground)
        {
            break;
        }
        row++;
    }

    int found_ground_row = row;  // Store the row number of the first pixel on the ground that was found
    
    /* Starting from the row of the first pixel with a valid depth value that was found, loop through rest of the 2D vector of Pixel objects 
       while coloring every pixel that could represent a ditch or gap in depth value between two objects blue in the image we're performing ditch detection on */
    if(found_pixel && found_ground)
    {
        Pixel curr_pixel;  // The pixel in the current row and column
        Pixel pixel_above;  // The pixel directly above curr_pixel
        Pixel pixel_to_right;  // The pixel directly to the right of curr_pixel
        
        double distance = 0;  // Variable used to store z distance between pixels
        
        // Loop through each column in LiDAR image
        for(int column = 0; column < 159; column++)
        {
            // Loop through each row (starting from the row of the first pixel with a valid depth value that was found)
            for(int curr_row = found_pixel_row; curr_row < 119; curr_row++)
            {
                // If the pixel in the current row is on the ground and has an invalid depth value, color the pixel blue
                if(curr_row >= found_ground_row && pixels[curr_row][column].z == -1)
                {
                    img.at<cv::Vec3b>(curr_row, column) = BLUE;  // Color the pixel blue
                    pixels[curr_row][column].color = "BLUE";
                    continue;
                }
                
                /* If the pixel in the current row is not on the ground and has an invalid depth value, search the rows above until a pixel (in the same column) with a valid depth value is found. 
                   From the row of this pixel, iterate back to the original row while coloring all the pixels in the column with invalid depths blue. 
                   This is done since all undetected pixels that are located below detected pixels in the LiDAR image, can be considered ditches */
                if(curr_row < found_ground_row && pixels[curr_row][column].z == -1)
                {
                    int row = curr_row - 1;

                    // Loop through rows above (in the same column) until a pixel with valid depth or a blue pixel is found
                    while(row >= found_pixel_row)
                    {
                        if(pixels[row][column].z != -1 || pixels[row][column].color == "BLUE")
                        {
                            row++;

                            // Loop back to original row while coloring all pixels with invalid depths blue
                            while(row <= curr_row)
                            {
                                img.at<cv::Vec3b>(row, column) = BLUE;  // Color the pixel blue
                                pixels[row][column].color = "BLUE";
                                row++;
                            }
                            break;  // Break outer while loop after reaching original row
                        }
                        row--;
                    }
                    continue;
                }

                /* If the current pixel isn't already blue, has a valid depth value, and the pixel in the row above also has a valid depth value, 
                   determine if there is a ditch or gap in depth value based on the z distance between these pixels */
                if(pixels[curr_row][column].color != "BLUE" && pixels[curr_row][column].z != -1 && pixels[curr_row - 1][column].z != -1)
                {
                    curr_pixel = pixels[curr_row][column];
                    transform(theta, curr_pixel.y, curr_pixel.z);  // Transform curr_pixel from ground's reference frame to LiDAR's reference frame
                    
                    pixel_above = pixels[curr_row - 1][column];
                    transform(theta, pixel_above.y, pixel_above.z);  // Transform pixel_above from ground's reference frame to LiDAR's reference frame
                    
                    distance = abs(curr_pixel.z - pixel_above.z);

                    // Color the current pixel blue if there's an unsafe z distance between it and the pixel above (the distance is unsafe if it's greater than the wheel's radius)
                    if(distance > WHEEL_RAD)
                    {
                        img.at<cv::Vec3b>(curr_row, column) = BLUE;  // Color the pixel blue
                        pixels[curr_row][column].color = "BLUE";
                        continue;
                    }
                }
                
                /* If the current pixel isn't already blue, has a valid depth value, and the pixel in the column to the right also has a valid depth value, 
                   determine if there is a ditch or gap in depth value based on the z distance between these pixels */
                if(pixels[curr_row][column].color != "BLUE" && pixels[curr_row][column].z != -1 && pixels[curr_row][column + 1].z != -1)
                {
                    curr_pixel = pixels[curr_row][column];
                    transform(theta, curr_pixel.y, curr_pixel.z);  // Transform curr_pixel from ground's reference frame to LiDAR's reference frame

                    pixel_to_right = pixels[curr_row][column + 1];
                    transform(theta, pixel_to_right.y, pixel_to_right.z);  // Transform pixel_to_right from ground's reference frame to LiDAR's reference frame
                    
                    distance = abs(curr_pixel.z - pixel_to_right.z);

                    // Color the current pixel blue if there's an unsafe z distance between it and the pixel to the right (the distance is unsafe if it's greater than the wheel's radius)
                    if(distance > WHEEL_RAD)
                    {
                        img.at<cv::Vec3b>(curr_row, column) = BLUE;  // Color the pixel blue
                        pixels[curr_row][column].color = "BLUE";
                        continue;
                    }
                }
            }
        }
    }
}





int main() {

    // Create black image
    cv::Mat img(120, 160, CV_8UC3, BLACK);

    
    // Initialize 2D vector of Pixel objects
    std::vector<std::vector<Pixel>> pixels(120, std::vector<Pixel>(160));
    
    
    double ground_slope = 0;  // Ground plane's angle of inclination in degrees (negative means tilted downwards and positive means tilted upwards)


    int found_pixel_row = 0;  // Row of first pixel in LiDAR image with a valid depth value (this will be determined after calling readInData() function)
    int found_pixel_col = 0;  // Column of first pixel in LiDAR image with a valid depth value (this will be determined after calling readInData() function)


    // Read in XYZ data and output ground plane's angle of inclination (ground_slope)
    readInData("absolute-path-to-x-values-file.csv", "absolute-path-to-y-values-file.csv", "absolute-path-to-z-values-file.csv", pixels, ground_slope);


    double theta = LIDAR_ANGLE - ground_slope;  // Angle in degrees by which ground reference frame must be rotated to be parallel with LiDAR reference frame (negative means tilted downwards and positive means tilted upwards)


    // Initialize variables to store pitch and roll angles
    double pitch_angle;
    double roll_angle;
    
    
    Pixel pixel;  // The current pixel the for loop below is on
    Pixel pitch_ref_pixel;  // The reference pixel for pitch (the current pixel's pitch angle with reference to this pixel will be determined)


    // Iterate through pixels in the LiDAR image (stored in the 2D vector of pixel objects), coloring the corresponding pixels in the original black image as appropriate
    for(int col = 0; col < 160; col++)
    {
        for(int row = 119; row >= 0; row--)
        {
            if(pixels[row][col].z != -1)  // Check if pixel has a valid depth value
            {   
                pixel = pixels[row][col];

                transform(theta, pixel.y, pixel.z);  // Transform pixel's coordinates from ground reference frame to LiDAR reference frame

                // Color the pixel gray if it's on the ground
                if(abs(pixel.y) <= GROUND)
                {
                    img.at<cv::Vec3b>(row, col) = GRAY;  // Color the pixel gray
                    pixels[row][col].color = "GRAY";
                    continue;
                }
                
                // Color the pixel green if it has a safe/traversible height
                else if(pixel.y > 0 && pixel.y <= SAFE)
                {
                    img.at<cv::Vec3b>(row, col) = GREEN;  // Color the pixel green
                    pixels[row][col].color = "GREEN";
                    continue;
                }
                

                // Calculate current pixel's pitch angle (in degrees) with reference to a nearby pixel (pitch_ref_pixel) below the current pixel and in the same column in the LiDAR image
                calculatePitch(pixels, row, col, pixel, pitch_ref_pixel, theta, pitch_angle);

                // Calculate roll angle (in degrees) between rover's left and right wheels assuming bottom centre of rover is located at current pixel's position in 3D space
                calculateRoll(pixels, col, pixel, theta, roll_angle);


                // If the pixel has too high of a pitch or roll angle, color it blue (if it lies below the ground) or red (if it lies above the ground)
                if(pitch_angle > MAX_PITCH || roll_angle > MAX_ROLL)
                {
                    // Color the pixel blue if it's below the ground
                    if(pixel.y < 0)
                    {
                        img.at<cv::Vec3b>(row, col) = BLUE;  // Color the pixel blue (to represent ditch)
                        pixels[row][col].color = "BLUE";
                    }

                    // Color the pixel red if it's above the ground
                    else
                    {
                        img.at<cv::Vec3b>(row, col) = RED;  // Color the pixel red
                        pixels[row][col].color = "RED";
                    }
                }

                // Else color the pixel yellow to represent a warning zone or slope that should be driven over with caution
                else
                {
                    img.at<cv::Vec3b>(row, col) = YELLOW;  // Color the pixel yellow
                    pixels[row][col].color = "YELLOW";
                }
            }
        }
    }


    // Detect ditches in image
    detectDitch(img, pixels, theta);


    // Output final image to JPG file
    cv::imwrite("C:/Users/Dew Bhaumik/Desktop/obstacle-avoidance/output.jpg", img);
    
    return 0;
}
