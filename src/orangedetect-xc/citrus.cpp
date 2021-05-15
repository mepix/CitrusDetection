//
//  citrus.cpp
//  orangedetect-xc
//
//  Created by Merrick Campbell on 5/13/21.
//

#include "citrus.hpp"


CitrusDetector::CitrusDetector(){
    
};


std::vector<CitrusDetector::Citrus> CitrusDetector::findFruit(cv::Mat& imgColor, cv::Mat& imgDepth){
    
    // Distance Filtering
    cv::Mat imgFGB = removeBackground(imgColor, imgDepth,-1,-1,10,false);
    cv::imshow("Foreground", imgFGB);
    
    // Color Filtering
    
    // Euclidean Clustering
    
    // Circular Hough Transform
    
    // RANSAC
    
    // Correlation
    
    
    return m_foundFruits;
}

//**//**//**//**//**//**//**//
// PRIVATE MEMBER FUNCTIONS //
//**//**//**//**//**//**//**//

cv::Mat CitrusDetector::removeBackground(cv::Mat& imgColor, cv::Mat& imgDepth, int kernelSize, int numItr, int threshBGD, bool useGrabCut){
    
    // Create Mat Containers
    cv::Mat imgBW, imgFGD;
    
    // Create 1 Channel Greyscale Image
    cv::cvtColor(imgDepth, imgBW, cv::COLOR_BGR2GRAY);
    
    // Create Mask for Image
    cv::Mat imgMask(imgBW.size(), CV_8UC1);
    
    // Decide Which Segmentation Algorithm to Use
    if(useGrabCut){
        // Create Foreground and Background Mask Regions
        cv::Mat imgMaskFGD = createMaskTDE(imgBW, 180, kernelSize);
        imgBW.setTo(255,imgBW == 0); // RS treats 0 as special, it is not near the camera!
        cv::Mat imgMaskBGD = createMaskTDE(imgBW, 90, kernelSize);
        
        // Create Mask with guesses for GrabCut
        imgMask.setTo(cv::Scalar::all(cv::GC_BGD)); // Set "background" as default guess
        imgMask.setTo(cv::GC_PR_BGD, imgMaskBGD == 0); // Relax this to "probably background" for pixels outside "far" region
        imgMask.setTo(cv::GC_FGD, imgMaskFGD == 255); // Set pixels within the "near" region to "foreground"

        // Apply the GrabCut Algorithm
        cv::Mat m_imgBackground;
        cv::Mat m_imgForeground;
        cv::grabCut(imgColor, imgMask, cv::Rect(), m_imgBackground, m_imgBackground,numItr,cv::GC_INIT_WITH_MASK);
        
        // Create New Images
        imgFGD = cv::Mat::zeros(imgColor.rows, imgColor.cols, CV_8UC3);
        imgColor.copyTo(imgFGD,(imgMask == cv::GC_FGD) | (imgMask == cv::GC_PR_FGD));
        
    } else { // Use Simple Filter
        imgFGD = cv::Mat::zeros(imgColor.rows, imgColor.cols, CV_8UC3);
        imgColor.copyTo(imgFGD,(imgBW > PIX_BLACK+threshBGD) );
    }
    
    return imgFGD;
}

cv::Mat CitrusDetector::createMaskTDE(cv::Mat& imgIn, int thresh, int edSize){
    // Create Structuring Element
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*edSize+1,2*edSize+1));
    
    // Threshold, Dilate, & Erode
    cv::Mat imgOut;
    cv::threshold(imgIn, imgOut, thresh, 255, cv::THRESH_BINARY);
    cv::dilate(imgOut, imgOut, kernel);
    cv::erode(imgOut, imgOut, kernel);
    
    return imgOut;
}
