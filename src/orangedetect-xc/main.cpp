//
//  main.cpp
//  orangedetect-xc
//
//  Created by Merrick Campbell on 5/12/21.
//

#include <iostream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include "example.hpp"          // Include short list of convenience functions for rendering
#include <chrono>

//#include <imgui.h>
//#include "imgui_impl_glfw.h"

// Includes for time display
#include <sstream>
#include <iostream>
#include <iomanip>

#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/imgproc/imgproc.hpp>

#include "rgbd.hpp"

bool useLiveStream = false;
std::string pathToBag = "/Users/merrickcampbell/Movies/realsense-bags/orange_groves/orange_1/20210428_091503.bag";


int main(int argc, char * argv[]) try
{
    std::cout << "Running Citrus Detector" << std::endl;

    CamRGBD citrusCam;
    citrusCam.setPathToBag(pathToBag);
    citrusCam.initStream(useLiveStream);
    
//    // Declare depth colorizer for pretty visualization of depth data
//    rs2::colorizer color_map;
//
//    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
//    rs2::pipeline pipe;
//
//    // Configure and start the pipeline
//    if (useLiveStream) {
//        pipe.start();
//
//    } else { // Load From Bag
//        rs2::config cfg;
//        cfg.enable_device_from_file(pathToBag);
//        cfg.enable_stream(RS2_STREAM_DEPTH);
//        cfg.enable_stream(RS2_STREAM_COLOR);
//        pipe.start(cfg); //File will be opened in read mode at this point
//    }
//
//    // RealSense Frame Alignment Tool
//    rs2::align alignFrames(RS2_STREAM_DEPTH);
////    rs2::align alignFrames(RS2_STREAM_COLOR);
    
    //using namespace cv;
    const auto windowDepth = "Depth Image";
    const auto windowColor = "Color Image";
    cv::namedWindow(windowDepth, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(windowColor, cv::WINDOW_AUTOSIZE);
    
    //while(true)
    while (cv::waitKey(5) < 0) // && getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
    {
        citrusCam.procPipe();
        cv::Mat imageDepth = citrusCam.getFrame(CamRGBD::frameType::RGBD_DEPTH);
        cv::Mat imageColor = citrusCam.getFrame(CamRGBD::frameType::RGBD_COLOR);
        
//        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
//        // Align all frames to depth viewport
//        data = alignFrames.process(data);
//        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
//        rs2::frame color = data.get_color_frame();
//
//        // Query frame size (width and height)
//        const int w = depth.as<rs2::video_frame>().get_width();
//        const int h = depth.as<rs2::video_frame>().get_height();
//
//        // Create OpenCV matrix of size (w,h) from the colorized depth data
//        cv::Mat imageDepth(cv::Size(w, h), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
//        cv::Mat imageColor(cv::Size(w, h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
//        cv::cvtColor(imageColor, imageColor, cv::COLOR_RGB2BGR);
        
        // Update the window with new data
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
