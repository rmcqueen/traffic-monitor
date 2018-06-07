# traffic-monitor

## Introduction
This project aims to reduce the manual labor associated with traffic surveying. The solution proposed uses an inexpensive Raspberry Pi 3 Board B+ with an attached Camera V2 module to perform vehicle counting, speed calculations, and data logging. This opens the possibility to identify areas of high speed violations and provides cost savings through the removal of human counters.


## Installation Instructions
1. Clone the github repository
2. Install the V4L dependency on your Raspberry Pi 

    ```sudo apt-get install v4l-utils```
3. Install the OpenCV library and GTest if you wish to run the unit tests 
4. Run the following command to activate the camera module

    ```sudo modprobe bcm2835-v4l2```
5. Make a build directory within the `traffic-monitor` root folder
6. Change directory to the build directory and use CMake to build the project

    ```cmake ..```
7. Move the produced executables to the root directory and run the generated traffic-monitor file

    ```mv traffic-monitor ..```

    ```./traffic-monitor```

    Note: you will need to modify the calibration region areas to correspond to your video's capture. These values can be changed within the `AppConfig.cpp` file.


## System Overview

The system is broken into four major components:
    
    1. Live video capture
        • Read video live from a Raspberry Pi Camera Module V2 @ 30 fps 
    2. Contour detection
        • For every frame read, perform background subtraction, filtering (blurring, opening/closing)
    3. Speed estimation
        • Create a region of known dimensions within the recorded frame to use as a calibration region. This region corresponds to some real world dimensions, allowing for precise speed calculations
        • Track when a vehicle first touches the calibration region, the moments within the region, and when the back of the vehicle touches the calibration region.
        • Estimation of the object's next position is used to track it between frames. Done by determining the direction and likely next position based on a Euclidean distance calculation.
    4. Data logging
        • Store all recorded video in H264 format. Allows for post-analysis of video to identify and faults within the system.
        • A log file is generated for every vehicle which has successfully passed through the calibration region. This log file contains a unique identifier for each vehicle and the associated speed.
        • Image snapshots are captured with the unique identifier as the image name (i.e, 0.jpg)

![High Level System Overview](https://github.com/rmcqueen/traffic-monitor/blob/master/docs/README/high_level.png)


## Results
Two case studies were performed in this work. The first being vehicles on a freeway and the second being vehicles on a residential road.

A general look at what a detected vehicle looks like is shown below:

![Detected Vehicle](https://github.com/rmcqueen/traffic-monitor/blob/master/docs/README/bounding_box.png)

1. Freeway
    
    • A sample video was used given by Chris Dahms (https://github.com/MicrocontrollersAndMore/OpenCV_3_Car_Counting_Cpp/blob/master/CarsDrivingUnderBridge.mp4)

    • Based on this video, 52 vehicles were within the region of interest of which 52 were correctly classified. This resulted in an accuracy of 100% for freeway vehicle tracking

    • The speed limit of the region in the video is 40 mp/h. The average speed of vehicles travelling through this region was 35 mp/h, presenting an accuracy of 87.5%

2. Residential

    • Live video was captured from a residential road within Canada. The video has been omitted for privacy of the author.

    • Based on this video, of the 5 vehicles which were recorded, 5 were correctly classified. This resulted in an accuracy of 100% for residential vehicle tracking.

    • The speed limit of the region in the video is 30 mp/h. The average speed of vehicles travelling through this region was 27.595 mp/h, presenting an accuracy of 91.98%.


An in-depth breakdown of this work can be found [here](https://github.com/rmcqueen/traffic-monitor/docs/paper.pdf)
