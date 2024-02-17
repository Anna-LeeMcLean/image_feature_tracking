# 2D Image Feature Tracking

This project implements feature detection, description and matching using various built-in algorithms in the OPENCV library to track keypoints
in a stream of ten (10) images. The images used are a sample from a car driving on a highway. Shi-Tomasi, Harris, BRISK, ORB, SIFT and AKAZE are implemented for feature detection while BRISK, ORB, SIFT, AKAZE, BRIEF and FREAK and implemented for feature description. The aim is to compare
all feature detector - feature descriptor pairs and determine the best three (3) which can later on be used for collision avoidance. Feature matching using Brute Force and FLANN are both implemented with options for nearest neighbour and k-nearest neighbour matching. David Lowe's distance ratio is used for filtering out the best matches.

## Detector-Descriptor Combo Camparison

The table below shows the data collected while running the algorithms for different combinations of feature detectors and the BRIEF descriptor. Of all the descriptors used, the BRIEF descriptor was found to have the fastest execution time while achieving the maximum number of keypoint matches. The average times taken for feature detection and description for the ten (10) images are provided in the table for each combination. The average number of keypoints and keypoint matches are also listed. Brute Force matching (with Hamming Distance calculation) and k-nearest neighbour were used for feature matching. Since the average number of keypoints and matches are the same, it is evident that this selection of algorithms for feature matching was able to find a corresponding match for each keypoint in a subsequent image. A ratio was calculated to rank the algorithm combinations. This ratio prioritizes speed in the detection and description processes while considering the number of keypoints to be processed. 

Ratio = (Average Detection Time + Average Description Time)   /   (Average # of Keypoints + Average # of Matches)

The faster the processing times and the larger the number of keypoints and matches, the smaller this ratio will be. The combinations are ranked from smallest ratio (best) to largest ratio (worst). Therefore, the fastest combination which processes the most keypoints will be ranked the highest. 

| Detector-Descriptor| Average Detection Time (ms)| Average Description Time (ms)| Average # of Keypoints| Averages # of Matches| Ratio | Overall Ranking
|--|--|--|--|--|--|--|
| FAST-BRIEF         |2.759                       |2.681                         |410.3                  |410.3                 |0.0066 |1
| ORB-BRIEF          |17.285                      |1.481                         |114.8                  |114.8                 |0.0817 |2
| SHITOMASI-BRIEF    |26.126                      |1.489                         |118.6                  |118.6                 |0.1164 |3 
| BRISK-BRIEF        |78.755                      |3.432                         |278.7                  |278.7                 |0.1474 |4 
| AKAZE-BRIEF        |121.568                     |1.2258                        |165.7                  |165.7                 |0.3705 |5 
| HARRIS-BRIEF       |20.074                      |0.8864                        |23.8                   |23.8                  |0.4403 |6
| SIFT-BRIEF         |200.876                     |0.963                         |138.8                  |138.8                 |0.7271 |7


The FAST-BRIEF combination is ranked highest since it has the fastest keypoint processing times even while processing the most keypoints. ORB-BRIEF and SHITOMASI-BRIEF are the next best respectively. Consider that the BRIEF descriptor had one of the fastest feature description times while working with SIFT keypoints but the SIFT-BRIEF combination is still the worst ranked. This is because the SIFT feature detection process was the slowest and therefore the combination's overall processing time was slow. Also, even though the HARRIS-BRIEF combination had some of the faster processing times, it is ranked lower because of the small number of keypoints it processed. 

## Installation Instructions

### Requirements
1. cmake >= 2.8

2. make >= 4.1 (Linux, Mac), 3.81 (Windows)

3. OpenCV >= 4.1
    * This must be compiled from source with the `opencv_contrib` module while using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT detector. 

4. gcc/g++ >= 9.4.0

5. C++ >= 11

### Local Install

`git clone https://github.com/Anna-LeeMcLean/image_feature_tracking.git`

`cd image_feature_tracking`

`mkdir build && cd build`

`cmake .. && make`

`./2D_feature_tracking`.