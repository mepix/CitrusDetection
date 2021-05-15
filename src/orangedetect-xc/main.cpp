//
//  main.cpp
//  orangedetect-xc
//
//  Created by Merrick Campbell on 5/12/21.
//

// Include RealSense Cross Platform API
#include <librealsense2/rs.hpp>

// Include OpenCV API
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Other Includes
#include <chrono>
#include <sstream>
#include <iostream>
#include <iomanip>

// Project Classes
#include "rgbd.hpp"
#include "citrus.hpp"

//TODO: Move These to input arguments before making the repo public
bool useLiveStream = false;
std::string pathToBag = "/Users/merrickcampbell/Movies/realsense-bags/orange_groves/orange_1/20210428_091503.bag";

int main(int argc, char * argv[]) try
{
    std::cout << "Running Citrus Detector" << std::endl;

    // Setup L515 Camera
    CamRGBD citrusCam;
    citrusCam.setPathToBag(pathToBag);
    citrusCam.initStream(useLiveStream);
        
    // Setup Citrus Detector
    CitrusDetector myOJ;
    

    // Create Windows for Visualization
    const auto windowDepth = "Depth Image";
    const auto windowColor = "Color Image";
    cv::namedWindow(windowDepth, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(windowColor, cv::WINDOW_AUTOSIZE);
    
    while (cv::waitKey(5) < 0)
    {
        // Get Current Frames
        citrusCam.procPipe();
        cv::Mat imageDepth = citrusCam.getFrame(CamRGBD::frameType::RGBD_DEPTH);
        cv::Mat imageColor = citrusCam.getFrame(CamRGBD::frameType::RGBD_COLOR);
                        
        // Find the Citrus
        if(!imageColor.empty() && !imageDepth.empty()){
            myOJ.findFruit(imageColor, imageDepth);
        }
        
        // Visualize
        cv::imshow(windowDepth, imageDepth);
        cv::imshow(windowColor, imageColor);
        
    }
    
    return EXIT_SUCCESS;
    
}

catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
