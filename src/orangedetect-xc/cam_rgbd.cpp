//
//  cam_rgbd.cpp
//  orangedetect-xc
//
//  Created by Merrick Campbell on 5/12/21.
//

#include "cam_rgbd.hpp"

CamRGBD::CamRGBD(){
    m_alignFrames = new rs2::align(RS2_STREAM_COLOR);
}

bool CamRGBD::initStream(bool useLiveStream){
    // Configure and start the pipeline
    if (useLiveStream) {
        m_pipe.start();
    } else { // Load From Bag
        m_cfg.enable_device_from_file(m_pathToBag);
        m_cfg.enable_stream(RS2_STREAM_DEPTH);
        m_cfg.enable_stream(RS2_STREAM_COLOR);
        m_pipe.start(m_cfg); //File will be opened in read mode at this point
    }
    
    // TODO: Add a real logic check here
    return true;
}

void CamRGBD::procPipe(){
    // Wait for next set of frames from the camera
    m_data = m_pipe.wait_for_frames();
    
    // Align all frames to depth viewport
    m_data = m_alignFrames->process(m_data);
    rs2::frame depth = m_data.get_depth_frame().apply_filter(m_colorMap);
    rs2::frame color = m_data.get_color_frame();
    
    // Query frame size (width and height)
    const int w = depth.as<rs2::video_frame>().get_width();
    const int h = depth.as<rs2::video_frame>().get_height();
    
    // Create OpenCV matrix of size (w,h) from the colorized depth data
    m_imgDepth = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
    m_imgColor = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(m_imgColor, m_imgColor, cv::COLOR_RGB2BGR);
}

cv::Mat* CamRGBD::getFrame(CamRGBD::frameType ftype){
    switch (ftype) {
        case CamRGBD::frameType::RGBD_COLOR:
            return &m_imgColor;
            break;
        case CamRGBD::frameType::RGBD_DEPTH:
            return &m_imgDepth;
        default:
            return new cv::Mat(cv::Size(1, 49), CV_64FC1, cv::Scalar(0));
            break;
    }
}


