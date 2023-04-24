# Results

In ***Scene 1 Real Image.jpg***, the makeshift ramp has a 30.4 degree 
angle of inclination, the cardboard box and paper towel roll represent tall obstacles with near 90 degree angles of inclination, and the ball of socks represents a rock of height 85 mm.

As seen in ***Scene 1 Detection.jpg***:

The ball of socks is colored grey because all of its pixels have absolute value heights less than or equal to 100 mm from the ground and therefore the ball of socks can be considered "ground".

The cardboard box and paper towel roll are both colored red for the most part because of their high angles of inclination. The pixels at the bottoms of both of these objects are colored green or gray because they have heights either between 100 mm and 150 mm (in which case they are considered "non-worrisome" and colored green), or below 100 mm (in which case they are considered "ground" and colored gray).

The top of the makeshift ramp is colored red because the ramp has a 30.4 degree slope, meaning that it's too dangerous to be driven over (it exceeds the 25 degree threshold). The rest of the ramp is colored green or gray because the pixels have heights either between 100 mm and 150 mm (in which case they are considered "non-worrisome" and colored green), or below 100 mm (in which case they are considered "ground" and colored gray).

The remaining blue pixels represent ditches/areas that should be considered "ground" but have invalid LiDAR detections. The *detectDitch()* function in ***detection.cpp*** colors these pixels blue.

In ***Scene 2 Real Image.jpg***, the cardboard box has a 20 degree angle of inclination.

As seen in ***Scene 2 Before Pixel Filtering.jpg***, the cardboard box is colored yellow for the most part because of its 20 degree slope (meaning it can be driven over with caution). However, due to sensor noise, many pixels on the cardboard box are colored red. After filtering the image with the pixel filtering algorithms in ***detection.cpp***, the new Scene 2 detection as seen in ***Scene 2 After Pixel Filtering.jpg***, significantly filters out this noise.

In ***Scene 3 Real Image.jpg***, there is a significant gap/crevasse between the white and brown cardboard box at the very back and the green and brown cardboard box directly in front of it. Although the LiDAR can't see this crevasse, as seen in ***Scene 3 Detection.jpg***, the *detectDitch()* function in ***detection.cpp*** is able to detect the crevasse and colors the gap between the cardboard boxes blue to show the crevasse.