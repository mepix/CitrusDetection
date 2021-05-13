//
//  cam_rgbd.hpp
//  orangedetect-xc
//
//  Created by Merrick Campbell on 5/12/21.
//

#ifndef cam_rgbd_hpp
#define cam_rgbd_hpp

#include <stdio.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

/**
 \brief CamRGBD is a lightweight wrapper around Intel's Realsense Library for the L515 library.
 */
class CamRGBD {
    
public:
    /**
     \brief frameType is an enum used to distinguish the type of RGBD frame
     */
    enum frameType{
        RGBD_COLOR,
        RGBD_DEPTH
    };
public:
    CamRGBD();
    /**
     \brief Initializes a real sense stream, either from a live camera or pre-recorded ROSbag
     \param useLiveStream set to TRUE to use the livestream from or set to FALSE to use a prerecorded ROSbag
     */
    bool initStream(bool useLiveStream);
    /**
     \brief Processes the current camera frame waiting in the image pipeline
     */
    void procPipe();
    /**
     \brief Retrieves a cv::Mat of the current frame
     \param ftype is an enum selector for the type of frame (COLOR or DEPTH)
     \returns A pointer to the selected image matrix
     */
    cv::Mat* getFrame(CamRGBD::frameType ftype);
    
private:
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer m_colorMap;
    
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline m_pipe;
    
    // Stream Config Settings
    rs2::config m_cfg;
    
    // RealSense Frame Alignment Tool
    rs2::align* m_alignFrames = NULL;
    //    rs2::align alignFrames(RS2_STREAM_COLOR);
    
    rs2::frameset m_data;
    
    cv::Mat m_imgColor;
    cv::Mat m_imgDepth;
    
    std::string m_pathToBag = "";
    
    
};

#endif /* cam_rgbd_hpp */
