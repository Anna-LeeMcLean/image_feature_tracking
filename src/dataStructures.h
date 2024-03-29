#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

enum class DetectorType {
    SHITOMASI,
    HARRIS, 
    FAST,
    BRISK,
    ORB,
    AKAZE,
    SIFT
};

enum class DescriptorType {
    BRISK,
    BRIEF, 
    ORB, 
    FREAK, 
    AKAZE, 
    SIFT
};

#endif /* dataStructures_h */
