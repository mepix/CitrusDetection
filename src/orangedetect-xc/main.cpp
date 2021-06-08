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

// Globals
bool useLiveStream = false;
std::string pathToBag = "";
bool record = false;
std::string saveFilePath = "";

// Main Routine
int main(int argc, char * argv[]) try
{
    std::cout << "Running Citrus Detector" << std::endl;

    // Process Input Arguments
    std::cout << argc << std::endl;
    switch(argc){
        case 3:
            std::cout << "Recording Enabled" << std::endl;
            record = true;
            saveFilePath = argv[2];
            std::cout << "Save File Path: " << saveFilePath << std::endl;
        case 2:
            std::cout << "Running from ROSBAG" << std::endl;
            useLiveStream = false;
            // Get the path to the ROSBAG
            pathToBag = argv[1];
            std::cout << "ROSBAG File Path: " << pathToBag << std::endl;

            break;
        case 1:
            std::cout << "Running from Live RGBD Camera" << std::endl;
            useLiveStream = true;
        // Fall through remaining cases
        case 0:
        default:
            std::cerr << "Invalid Input Arguments" << std::endl;
            return EXIT_FAILURE;
            break;
    };
    
    
    // Setup L515 Camera
    CamRGBD citrusCam;
    citrusCam.setPathToBag(pathToBag);
    citrusCam.initStream(useLiveStream);
        
    // Setup Citrus Detector
    CitrusDetector myOJ(true);
    myOJ.setSaveFilePath(saveFilePath);
    if(record){myOJ.startRecord();}
    

    // Create Windows for Visualization
    const auto windowDepth = "Depth Image";
    const auto windowColor = "Color Image";
    cv::namedWindow(windowDepth, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(windowColor, cv::WINDOW_AUTOSIZE);
    
    while (cv::waitKey(1) < 0)
    {
        // Get Current Frames
        citrusCam.procPipe();
        cv::Mat imageDepth = citrusCam.getFrame(CamRGBD::frameType::RGBD_DEPTH);
        cv::Mat imageColor = citrusCam.getFrame(CamRGBD::frameType::RGBD_COLOR);
                                        
        // Find the Citrus
        if(!imageColor.empty() && !imageDepth.empty()){
            myOJ.findFruit(imageColor, imageDepth, CitrusDetector::CITRUS_ORANGE,true);
        }
        
        // Visualize
        cv::imshow(windowDepth, imageDepth);
        cv::imshow(windowColor, imageColor);
        
    }
    
    // Stop Recording
    myOJ.stopRecord();
    
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
