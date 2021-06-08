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

#define DEBUG_CLUSTER false

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
        std::vector<cv::Point2f> centers;
        std::vector<float> radius;
    };
protected:
    enum citrusCluster{
        CLUSTER_2D,
        CLUSTER_3D
    };
private:
    // Vector for Found Fruits
    CitrusDetector::Citrus m_foundFruits;
    
    // Fruit Color Descriptors
    int m_fruitColorOrange = 15;
    int m_fruitColorGrapefruit = 30;
    int m_fruitColorRange = 7;
    
    // Fruit Spatial Descriptors
    float m_fruitMinRadius = 7.5;
    
    // Filter Kernels
    int m_depthFilterKernelSize = 5;
    int m_morphFilterKernelSize = 5; //This has a big role in the number of citrus detected
    int m_depthNumOpeningItr = 3;
    
    // Image Scaling
    bool m_scaleInput = true;
    double m_scaleFactor = 0.25;
    
    // Clustering Thresholds
    int m_clusterThresh3D = 24;
    int m_clusterThresh2D = 18;

    
public:
    CitrusDetector(bool scaleInput);
    CitrusDetector::Citrus findFruit(cv::Mat& imgColor, cv::Mat& imgDepth, CitrusDetector::citrusType fruitType, bool visualize); //TODO: change to vector of fruits
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
    
    /**
     \brief Filters the input image by the target color hue and removes all pixels not within the target range
     \param img the frame to be filtered
     \param targetColor the hue to be kept during the filtering operation [0-179]
     \param targetRange the band (+/-) around the target range ro be kept durign the filtering  operation
     \return a new filtered image matrix
     */
    cv::Mat colorFilter(cv::Mat& img, int targetColor, int targetRange);
    
    /**
     \brief Converts the fruitType enum to a Hue value for the desired fruit
     \param type indicates the desired citrus type
     \return the hue value corresponding to the specific fruit, [0-179]
     */
    int getTargetColorFromFruitType(citrusType type);
    
    cv::Mat morphOpen(cv::Mat& img, int numItr);
    
    
    std::vector<std::vector<cv::Point>> clusterFruits(cv::Mat& imgColor, cv::Mat& imgDepth, cv::Mat3b& imgClusterOut,CitrusDetector::citrusCluster clusterMethod);
    
    
    /**
     \brief Fits circles to the fruit using a collection of clustered blobs
     \param contours is a collection of points that represent each blob
     \return a Citrus struct that includes vectors of fruit centers and radiuses
     */
    CitrusDetector::Citrus fitCirclesToFruit(std::vector<std::vector<cv::Point>> contours);
    
    /**
     \brief Removed objets that are too small to be considered a fruit
     \param fruit is a struct that contains the centers and radisus of fruit candidates
     \param radiusThresh indicated the minimum radius necessary to be considered a fruit
     */
    bool filterFoundFruit(CitrusDetector::Citrus& fruit, float radiusThresh);
    
    
    /**
     \brief Draws circles on the image that represent the detected fruits
     \param img for the circles to be drawn on, this image will be modified
     \param fruit a collection of fruist with a center point and radius
     */
    void drawFruitCircles(cv::Mat& img, CitrusDetector::Citrus fruit);
    
    cv::Mat createTextBar(int numCitrusFound, int fps, cv::Size size, bool top);
    
    
};


#endif /* citrus_hpp */
