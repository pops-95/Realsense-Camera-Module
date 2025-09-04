#include <iostream>
#include <opencv2/opencv.hpp>
#include "Utils.hpp"

int main(int argc, char *argv[])
{
    rs2::context ctx;
    auto devices = ctx.query_devices();
    std::vector<Camera> cameras;

    // Set desired resolution and fps
    color_frame_info color_info = {1280, 720, 30};
    depth_frame_info depth_info = {1280, 720, 30};

    for (size_t i = 0; i < devices.size(); ++i) {
        rs2::device dev = devices[i];
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::string name = "Camera_" + serial;
        cameras.emplace_back(serial, name);
        cameras.back().change_color_frame(color_info);
        cameras.back().change_depth_frame(depth_info);
        cameras.back().enable_streams();
        cameras.back().start_camera();
        std::cout << "Created camera object for device: " << name << std::endl;
    }

    if (cameras.empty()) {
        std::cerr << "No RealSense cameras detected." << std::endl;
        return -1;
    }

    // Show color and thresholded depth images from the first camera
    uint16_t threshold_value = 1000; // Example: 2000 units (usually mm, i.e. 2 meters)

    while (true) {
        // Get frames from camera
        rs2::frameset frames = cameras[0].get_frames();
        rs2::frameset aligned = cameras[0].aligned_frames(frames);

        // Get color image
        cv::Mat color_img = cameras[0].get_rgb_image(aligned);

        // Get depth frame
        rs2::depth_frame depth_frame = aligned.get_depth_frame();

        cv::Mat depth_thresh_img;
        if (depth_frame)
            depth_thresh_img = cameras[0].threshold_depth_frame(depth_frame, threshold_value);

        // Optionally normalize for display
        cv::Mat depth_display;
        if (!depth_thresh_img.empty())
            cv::normalize(depth_thresh_img, depth_display, 0, 255, cv::NORM_MINMAX, CV_8U);

        if (!color_img.empty())
            cv::imshow("Color Image", color_img);
        if (!depth_display.empty())
            cv::imshow("Depth Image (Thresholded)", depth_display);

        if (cv::waitKey(1) == 27) // ESC key
            break;
    }

    cv::destroyAllWindows();
    return 0;
}
