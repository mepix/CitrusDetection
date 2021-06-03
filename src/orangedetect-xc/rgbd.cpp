//
//  rgbd.cpp
//  orangedetect-xc
//
//  Created by Merrick Campbell on 5/12/21.
//

#include "rgbd.hpp"

CamRGBD::CamRGBD(){
    // Create Alignment Utility
    m_alignFrames = new rs2::align(RS2_STREAM_COLOR);
    //m_alignFrames = new rs2::align(RS2_STREAM_DEPTH);
    
    // Set ColorMap Scheme
    m_colorMap.set_option(RS2_OPTION_COLOR_SCHEME, 2);
}

CamRGBD::~CamRGBD(){
    if(m_streamStarted){
        m_pipe.stop();
    }
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
        
        //TODO:MPC:BUG: This set_real_time does not seem to work and freezes all playback!
        // set_real_time = false to make sure we process every frame from the bag
//        auto device = m_pipe.get_active_profile().get_device();
//        rs2::playback playback = device.as<rs2::playback>();
//        playback.set_real_time(true);
//        playback.resume();
//        auto duration = playback.get_duration();
        
    }
    

    
    m_streamStarted = true;
    
    // TODO: Add a real logic check here
    return true;
}

void CamRGBD::procPipe(){
    // Wait for next set of frames from the camera
    m_data = m_pipe.wait_for_frames();
//    m_pipe.poll_for_frames( &m_data );
    
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

cv::Mat CamRGBD::getFrame(CamRGBD::frameType ftype){
    switch (ftype) {
        case CamRGBD::frameType::RGBD_COLOR:
            return m_imgColor;
            break;
        case CamRGBD::frameType::RGBD_DEPTH:
            return m_imgDepth;
        default:
            return cv::Mat(cv::Size(1, 49), CV_64FC1, cv::Scalar(0));
            break;
    }
}

void CamRGBD::setPathToBag(std::string path){
    m_pathToBag = path;
}
