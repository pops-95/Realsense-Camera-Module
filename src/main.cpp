#include <iostream>
#include "Utils.hpp"

int main(int argc, char *argv[])
{
    rs2::context ctx;
    auto devices = ctx.query_devices();
    std::vector<Camera> cameras;

    for (size_t i = 0; i < devices.size(); ++i) {
        rs2::device dev = devices[i];
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::string name = "Camera_" + serial;
        cameras.emplace_back(serial, name);
        std::cout << "Created camera object for device: " << name << std::endl;
    }

    if (cameras.empty()) {
        std::cerr << "No RealSense cameras detected." << std::endl;
    }

    // ... use cameras vector as needed ...
 

    return 0;
}
