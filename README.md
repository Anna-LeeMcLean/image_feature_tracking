# 2D Image Feature Tracking

This project implements feature detection, description and matching using various built-in algorithms in the OPENCV library. 
The ten (10) images used are a sample from a car driving on a highway. Keypoints are detected in the first image then tracked
throughout the remaining nine (9). Shi-Tomasi, Harris, BRISK, ORB, SIFT and AKAZE are implemented for feature detection while
BRISK, ORB, SIFT, AKAZE, BRIEF and FREAK and implemented for feature description. Feature matching using Brute Force and FLANN
are both implemented with options for nearest neighbour and k-nearest neighbour matching. The Lowe's distance ratio is used for 
determing the best matches.
