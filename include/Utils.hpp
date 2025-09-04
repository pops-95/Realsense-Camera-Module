#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <iostream>

typedef struct{
    int depth_width;
    int depth_height;
    int depth_fps;
}depth_frame_info;

typedef struct{
    int color_width;
    int color_height;
    int color_fps;
}color_frame_info;


class Camera{
    public:
        Camera(const std::string& serial, const std::string& name);
        ~Camera();
        Camera (color_frame_info color_info, depth_frame_info depth_info);
        void enable_streams();
        void start_camera();
        void get_serial(std::string& serial_out) { serial_out = serial; }
        void change_depth_frame(depth_frame_info& depth_info); 
        void change_color_frame(color_frame_info& color_info);
        
        rs2::frameset aligned_frames();

        void set_exposure(int value, bool auto_exposure);

        void get_color_frame_info(color_frame_info& color_info_out) { color_info_out = color_info; }
        void get_depth_frame_info(depth_frame_info& depth_info_out) { depth_info_out = depth_info; }
        
    private:
        rs2::pipeline pipe;
        rs2::config cfg;
        rs2::context ctx;
        color_frame_info color_info;
        depth_frame_info depth_info;
        std::string serial;
        std::string name;
        rs2::depth_frame depth_frame;
        rs2::frameset get_frames();

        
};

