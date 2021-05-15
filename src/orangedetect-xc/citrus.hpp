//
//  citrus.hpp
//  orangedetect-xc
//
//  Created by Merrick Campbell on 5/13/21.
//

#ifndef citrus_hpp
#define citrus_hpp

#include <stdio.h>
#include <vector>


#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define PIX_BLACK 0
#define PIX_WHITE 255

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
private:
    std::vector<CitrusDetector::Citrus> m_foundFruits;

    
public:
    CitrusDetector();
    std::vector<CitrusDetector::Citrus> findFruit(cv::Mat& imgColor, cv::Mat& imgDepth); //TODO: change to vector of fruits
    cv::Mat drawFruit(cv::Mat& img); // TODO: add an argument for a vector of citrus fruits
private:
    /**
     \brief Segments the foreground and background using the depth imformation.
     \param imgColor 3-channel color image
     \param imgDepth 3-channel greyscale depth image
     \param kernelSize for use in the cv::GrabCuts algorithm
     \param numItr for use in the cv::GrabCuts algorithm
     \param threshBGD background threshold value for simple threshold
     \param useGrabCut [T] to use the cv::GrabCut and [F] to use a simple threshold
     \return a color matrix of the foreground pixels
     */
    cv::Mat removeBackground(cv::Mat& imgColor, cv::Mat& imgDepth, int kernelSize, int numItr, int threshBGD, bool useGrabCut);
    /**
     \brief Performs a threshold, dilation, and erosion operation to create a mask
     \param imgIn 1-channel greyscale image
     \param thresh threshold value
     \param edSize erosion and dilation kernel size
     \return the created mask
     */
    cv::Mat createMaskTDE(cv::Mat& imgIn, int thresh, int edSize);
    
    cv::Mat colorFilter(cv::Mat& imgFGD, int targetColor);
};


#endif /* citrus_hpp */
