### Introduction
This project aims to incorporate the widely available and open-sourced Raspberry Pi hardware to monitor the  traffic around the University of British Columbia Okanagan campus. With this information, data analysis can be conducted in order to gauge if there are traffic issues on the campus, prompting a potential solution to be constructed in order alleviate the issue.

### Problem
With the desire of living within the Okanagan and the increasing prestige of the University of British Columbia, the enrolment numbers continue to climb at the University of British Columbia. With these increasing enrolment numbers, more cars begin to appear on campus, additional parking lots are required to alleviate parking issues, and an overall increase in traffic has been noticed. This project aims to identify areas of high congestion on campus and provide the authority in charge of traffic solutions with a cheap, accurate, and robust traffic monitoring system.

### Existing Work
There has been a marginal amount of published research work in the area of utilizing Raspberry Pi microcomputers for the usage of traffic monitoring. The existing work includes approaches such as using background subtraction in order to identify the cars, object detection for a parking garage and attempting to index each unique car, and looking at the radio signals given off by a doppler module in order to identify a vehicle.

There approaches listed above lack functionality in environments with a lot of noise (such as people walking or a large amount of greenery) and as a result, will fail to accurately detect and record traffic.  

### Timeline
This project is broken into four major components:

1. Existing Work Research
    * The ideas already considered with their associated pros/cons
    * Constraints
    * Recommendations
2. Hardware Configuration
    * Configure Raspberry Pi board and camera
    * Hello World application
3. Implementation
    * Snapshots of the road every ~3 seconds with low resolution (save space/time)
    * Object detection algorithms for the cars
    * Car speed estimation
4. Field Testing
    * Can it predict cars with a high accuracy (> 90%)?
    * Does the speed estimation look reasonable?
    * Unexpected results

### Resources
* Hardware
    1. Raspberry Pi 3 Model B
    2. Raspberry Pi Camera Module V2
    3. Custom made waterproof enclosure for the Raspberry Pi
* Software
    1. C++ programming language
    2. OpenCV library (C++ version)