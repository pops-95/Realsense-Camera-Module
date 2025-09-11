// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <iostream>
#include <Utils.hpp>
// #include "example.hpp"              // Include short list of convenience functions for rendering

#include <map>
#include <vector>

int main(int argc, char * argv[]) 
{
    // Create a simple OpenGL window for rendering:
    // window app(1280, 960, "CPP Multi-Camera Example");

    rs2::context                          ctx;        // Create librealsense context for managing devices

    std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)

    std::vector<rs2::pipeline>            pipelines;

    // Capture serial numbers before opening streaming
    std::vector<std::string>              serials;
    std::vector<Camera>                   cameras;
    std::vector<camera_frames>            frames_vec;

    // Example: let’s allow user choice
    color_frame_info cinfo = {640, 480, 30};  // default
    depth_frame_info dinfo = {640, 480, 30};  // default
    // Camera cam;
    for (auto&& dev : ctx.query_devices())
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    // Start a streaming pipe per each connected device
    for (auto&& serial : serials)
    {
        
        rs2::config cfg;
      
        cfg.enable_device(serial);
        // cfg.enable_stream(RS2_STREAM_COLOR, cinfo.color_width, cinfo.color_height, RS2_FORMAT_BGR8, cinfo.color_fps);
        // cfg.enable_stream(RS2_STREAM_DEPTH, dinfo.depth_width, dinfo.depth_height, RS2_FORMAT_Z16, dinfo.depth_fps);
        cameras.emplace_back(serial,"Camera_"+serial,cinfo,dinfo,ctx,cfg);
        
       
        frames_vec.emplace_back(serial);
        // serials.push_back(serial);
        std::cout << "Started streaming from device " << serial << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }

    // for (size_t i = 0; i < cameras.size(); i++) {
    // if (!cameras[i].verify_pipeline_serial()) {
    //     std::cerr << "Camera " << i << " pipeline association is invalid!" << std::endl;
    // }
    //     std::cerr << "Camera " <<cameras[i].serial << std::endl;
    // }

    // // For each pipeline, get the associated device and print its serial
    // for (auto &pipe : pipelines){
    //      try {
    //     std::string p_serial = pipe.get_active_profile()
    //                                .get_device()
    //                                .get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    //     std::cout << "Pipeline associated with device " << p_serial << std::endl;
    // } catch (const rs2::error& e) {
    //     std::cerr << "Failed to get active profile: " << e.what() << std::endl;
    // }
    //   }
 

    // // Main app loop
    while (true)
    {
      
        for (int i=0;i<2;i++){
             rs2::frameset fs=cameras[i].get_frames(); 
              if (fs.size() > 0){
                    std::cout<< "Got frames from camera " << cameras[i].serial << std::endl;
                    cameras[i].camera_operation(fs,cameras[i], frames_vec[i]);
              }
        }

        // Show both RGB images in separate windows
        if (!frames_vec[0].color_image.empty())
            cv::imshow("Camera 1 - RGB", frames_vec[0].color_image);

        if (!frames_vec[1].color_image.empty())
            cv::imshow("Camera 2 - RGB", frames_vec[1].color_image);

        // Update every frame, press ESC to exit
        if (cv::waitKey(1) == 27) {  
            break;
        }
       
    }

    return EXIT_SUCCESS;
}





