#include "Utils.hpp"

Camera::Camera(const std::string &serial, const std::string &name)
{
    // Default Constructor
    this->serial = serial;
    this->name = name;
}

void Camera::enable_streams()
{
    // Enable the streams
    cfg.enable_stream(RS2_STREAM_DEPTH, depth_info.depth_width, depth_info.depth_height, RS2_FORMAT_Z16, depth_info.depth_fps);
    cfg.enable_stream(RS2_STREAM_COLOR, color_info.color_width, color_info.color_height, RS2_FORMAT_BGR8, color_info.color_fps);
}

void Camera::start_camera()
{
    // Start the camera
    pipe.start(cfg);
}

Camera::~Camera()
{
    // Destructor
    pipe.stop();
}

Camera::Camera(color_frame_info color_info, depth_frame_info depth_info)
{
    this->color_info = color_info;
    this->depth_info = depth_info;
}

void Camera::change_depth_frame(depth_frame_info &depth_info)
{
    this->depth_info = depth_info;
}

void Camera::change_color_frame(color_frame_info &color_info)
{
    this->color_info = color_info;
}

rs2::frameset Camera::get_frames()
{
    try
    {
        return pipe.wait_for_frames();
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error calling wait_for_frames(): " << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Standard exception calling wait_for_frames(): " << e.what() << std::endl;
    }
    // Return an empty frameset if an error occurs
    return rs2::frameset();
}

rs2::frameset Camera::aligned_frames()
{
    rs2::frameset frames = get_frames();
    if (frames.size() == 0)
    {
        std::cout << "No frames " << std::endl; // Return empty frameset if no frames are available
    }
    rs2::align align_to_color(RS2_STREAM_COLOR);
    rs2::frameset aligned_frames = align_to_color.process(frames);
    return aligned_frames;
}

void Camera::set_exposure(int value, bool auto_exposure)
{
    // Get device from pipeline
    rs2::pipeline_profile profile = pipe.get_active_profile();
    rs2::device dev = profile.get_device();

    // Find the first sensor that supports color stream
    for (auto &&sensor : dev.query_sensors())
    {
        if (sensor.supports(RS2_OPTION_EXPOSURE))
        {
            if (auto_exposure)
            {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
            }
            else
            {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
                sensor.set_option(RS2_OPTION_EXPOSURE, static_cast<float>(value));
            }
        }
    }
}


