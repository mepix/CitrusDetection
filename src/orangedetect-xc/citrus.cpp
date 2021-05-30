//
//  citrus.cpp
//  orangedetect-xc
//
//  Created by Merrick Campbell on 5/13/21.
//

#include "citrus.hpp"


CitrusDetector::CitrusDetector(){
    
};


CitrusDetector::Citrus CitrusDetector::findFruit(cv::Mat& imgColor, cv::Mat& imgDepth, CitrusDetector::citrusType fruitType, bool visualize){
    
    // Distance Filtering
    cv::Mat imgDistFiltered = removeBackground(imgColor, imgDepth,-1,-1,10,false);
    
    // Color Filtering
    int color = getTargetColorFromFruitType(fruitType);
    cv::Mat imgColorFiltered = colorFilter(imgDistFiltered, color,m_fruitColorRange);
    
    // Euclidean Clustering
    cv::Mat imgClustered = cv::Mat::zeros(imgColor.rows,imgColor.cols,CV_8UC3);
    std::vector<std::vector<cv::Point>> contours = clusterFruits(imgColorFiltered);
    
    // Circle Fitting
    m_foundFruits = fitCirclesToFruit(contours);
    
    // RANSAC
    
    // Correlation
    
    // Visualize

    if (visualize){
        // Draw Fruits on Images
        drawFruitCircles(imgColor,m_foundFruits);
        drawFruitCircles(imgColorFiltered, m_foundFruits);

        // Create Combo Image to Show
        cv::Mat row1,row2,imgVis;
        cv::hconcat(imgColor, imgDepth, row1);
        cv::hconcat(imgColorFiltered, imgDistFiltered, row2);
        cv::vconcat(row1, row2, imgVis);
        cv::imshow("Detected Fruit",imgVis);
    }
    
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
        
    } else { // Use Simple Threshold Filter
        // Perform an Opening Operation (Erode, then Dilate)
        cv::Mat imgBlur, imgErode, imgDilate;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
        int numOpeningItr = 3;
        
        // Smooth
        cv::GaussianBlur(imgBW, imgBlur, cv::Size(5,5), 0.5);
        
        // Erode
        cv::erode(imgBlur, imgErode, kernel,cv::Point(-1, -1),numOpeningItr);
        
        // Dilate
        cv::dilate(imgErode, imgDilate, kernel,cv::Point(-1, -1),numOpeningItr);
        
        // Apply Mask
        imgFGD = cv::Mat::zeros(imgColor.rows, imgColor.cols, CV_8UC3);
        imgColor.copyTo(imgFGD,(imgDilate > PIX_BLACK+threshBGD) );
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

cv::Mat CitrusDetector::colorFilter(cv::Mat& img, int targetColor, int targetRange){
    // Create HSV image
    cv::Mat imgHSV;
    cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV);
    
    // Get the H channel
    std::vector<cv::Mat> channelsHSV(3);
    cv::split(imgHSV, channelsHSV);
    cv::Mat hue = channelsHSV[0]; // H is the first channel
    
    // Get the G channel
    std::vector<cv::Mat> channelsBGR(3);
    cv::split(img, channelsBGR);
    cv::Mat green = channelsBGR[1]; // G is the second channel
    
    // Create Mask
    cv::Mat imgMask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
    imgMask.setTo(PIX_WHITE,(hue > targetColor - targetRange) & (hue < targetColor + targetRange)); // Select Citrus Color Pixels from Hue Space
    imgMask.setTo(PIX_BLACK,(green > 230)); // Remove Green Leaf Traces
    
    // Open Mask (Smooth, Erode, Dilate)
    imgMask = morphOpen(imgMask, 3);
    
    // Throwaway values outside range
    cv::Mat imgFruit = cv::Mat::zeros(img.rows,img.cols, CV_8UC3);
    img.copyTo(imgFruit,imgMask);
    return imgFruit;
}

int CitrusDetector::getTargetColorFromFruitType(citrusType type){
    switch (type) {
        case CITRUS_ORANGE:
            return m_fruitColorOrange;
            break;
        case CITRUS_GRAPEFRUIT:
            return m_fruitColorGrapefruit;
            break;
        default:
            std::cerr << "Invalid Citrus Type in getTargetColorFromFruitType" << std::endl;
            return 90; // Value Halfway between [0-179]
            break;
    }
}


cv::Mat CitrusDetector::morphOpen(cv::Mat& img, int numItr){
    // Create Mats
    cv::Mat imgBlur, imgErode, imgDilate;
    
    // Get Kernel
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
    
    // Smooth
    cv::GaussianBlur(img, imgBlur, cv::Size(5,5), 0.5);
    
    // Erode
    cv::erode(imgBlur, imgErode, kernel,cv::Point(-1, -1),numItr);
    
    // Dilate
    cv::dilate(imgErode, imgDilate, kernel,cv::Point(-1, -1),numItr);
    
    return imgDilate;
}

std::vector<std::vector<cv::Point>> CitrusDetector::clusterFruits(cv::Mat& img){
    //    cv::imshow("Temp",mask);
    cv::Mat mask;
    cv::cvtColor(img,mask,cv::COLOR_BGR2GRAY);
    //    cv::imshow("Temp2",mask);
    //    cv::waitKey(0);
    //
    // Get all non black points
    std::vector<cv::Point> pts;
    cv::findNonZero(mask, pts);
    
    // Define the radius tolerance
    int th_distance = 18; // radius tolerance
    
    // Apply partition
    // All pixels within the radius tolerance distance will belong to the same class (same label)
    std::vector<int> labels;
    
    int th2 = th_distance * th_distance;
    int n_labels = cv::partition(pts, labels, [th2](const cv::Point& lhs, const cv::Point& rhs) {
        return ((lhs.x - rhs.x)*(lhs.x - rhs.x) + (lhs.y - rhs.y)*(lhs.y - rhs.y)) < th2;
    });
    
    // You can save all points in the same class in a vector (one for each class), just like findContours
    std::vector<std::vector<cv::Point>> contours(n_labels);
    for (int i = 0; i < pts.size(); ++i)
    {
        contours[labels[i]].push_back(pts[i]);
    }
    
    // Draw results
    
    // Build a vector of random color, one for each class (label)
    std::vector<cv::Vec3b> colors;
    for (int i = 0; i < n_labels; ++i)
    {
        colors.push_back(cv::Vec3b(rand() & 255, rand() & 255, rand() & 255));
    }
    
    // Draw the labels
    cv::Mat3b lbl(mask.rows, mask.cols, cv::Vec3b(0, 0, 0));
    for (int i = 0; i < pts.size(); ++i)
    {
        lbl(pts[i]) = colors[labels[i]];
    }
    
    imshow("Labels", lbl);
    //    cv::waitKey();
    
    return contours;
}

CitrusDetector::Citrus CitrusDetector::fitCirclesToFruit(std::vector<std::vector<cv::Point>> contours){
    
    // Declare Variables
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<cv::Point2f>centers( contours.size() );
    std::vector<float>radius( contours.size() );
    
    // Loop through the contours
    for( int i = 0; i < contours.size(); i++ )
    {
        // Use a polygon approximation for the contour blobs
        cv::approxPolyDP( contours[i], contours_poly[i], 3, true );
        
        // Find the minimum enclosing circle
        cv::minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
    }
    
    // Prepare a citrus struct
    Citrus fruitCircles;
    fruitCircles.centers = centers;
    fruitCircles.radius = radius;
    
    // Return the citrus struct
    return fruitCircles;
}

void CitrusDetector::drawFruitCircles(cv::Mat& img, CitrusDetector::Citrus fruit){
    
    // Color to draw the circles (Purple)
    cv::Scalar color = cv::Scalar(255,0,127);
    
    // Iterate through the fruit
    for( int i = 0; i< fruit.centers.size(); i++ )
    {
        // Draw the Circles
        cv::circle( img, fruit.centers[i], (int)fruit.radius[i], color, 5 );
        cv::drawMarker(img, fruit.centers[i], color);
    }
    
}
