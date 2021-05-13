//
//  citrus.hpp
//  orangedetect-xc
//
//  Created by Merrick Campbell on 5/13/21.
//

#ifndef citrus_hpp
#define citrus_hpp

#include <stdio.h>

#include <opencv2/opencv.hpp>   // Include OpenCV API

/**
 \brief CitrusDetector is a C++ class that uses classical computer vision methods to detect citrus fruit on trees in an orchard environment.
 */
class CitrusDetector {
public:
    enum citrusType{
        CITRUS_ORANGE,
        CITRUS_GRAPEFRUIT
    };
    /**
     \brief Citrus is a lighweight structure that defines a fruit with it's center coordinate and radius (in pixel space)
     */
    struct Citrus{
        cv::Point center;
        int radius;
    };
    
public:
    CitrusDetector();
    CitrusDetector::Citrus findFruit(cv::Mat imgColor, cv::Mat imgDepth); //TODO: change to vector of fruits
    cv::Mat drawFruit(cv::Mat img); // TODO: add an argument for a vector of citrus fruits
private:
};


#endif /* citrus_hpp */
