# LiDAR Obstacle Detection

Please refer to ***documentation.pdf*** file for in-depth algorithm explanation/documentation.

Before running the code, please ensure that the latest versions of OpenCV and CMake are installed and configured properly.

Modify the paths in lines 833 and 929 in ***detection.cpp*** as appropriate (absolute path means path relative to root directory of OS).

To build and run the code, enter the following commands:

```mkdir build && cd build```

```cmake ..```

```make```

```./obstacle-avoidance```

## Other Notes
Please read the parameters (eg. *#define HEIGHT*) at the beginning of the ***detection.cpp*** file and modify them as necessary.

In the *main()* function in ***detection.cpp***, by default a median filter is applied to the LiDAR image followed by a gaussian filter. Feel free to play around with the three pixel filtering functions in the code and uncomment/comment them out as necessary in the *main()* function.