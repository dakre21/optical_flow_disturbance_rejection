/*
Author: David Akre
Date: 2/17/24
Description: Main source code for polling cameras (camera module3 and luxonis cameras), computing optical flow, and storing results in shared memory
*/

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <assert.h>
#include <csignal>

#include "camera_module3.h"
#include "luxonis_camera.h"

std::atomic<bool> running(true);

void SignalHandler(int signum)
{
    (void)signum;
    std::cout << "Ctrl-C Detected. Stopping Threads" << std::endl;
    running = false;
}

int main(int argc, char *argv[])
{
    (void)argc, (void)argv;
    const int width = 640, height = 480;
    const int frame_rate = 140;
    constexpr const char *camera_type = "imx708";

    std::signal(SIGINT, SignalHandler);

    libcamera::CameraManager camera_manager;
    if (camera_manager.start())
    {
        throw std::logic_error("Failed to start camera manager");
    }
    const auto cameras = camera_manager.cameras();
    if (cameras.empty())
    {
        throw std::logic_error("Failed to find cameras on the Pi");
    }

    std::vector<std::unique_ptr<camera_driver::CameraBase>> camera_modules;
    int idx = 0;
    for (auto cam : cameras)
    {
        const auto camera_id = cam->id();
        if (camera_id.find(camera_type) == std::string::npos)
        {
            continue;
        }
        std::cout << "Camera id: " << camera_id << std::endl;
        auto camera = camera_manager.get(camera_id);
        camera_modules.emplace_back(
            std::make_unique<camera_driver::CameraModule3>(camera, width, height, std::to_string(idx++), running));
    }

    for (const auto &dev : dai::Device::getAllAvailableDevices())
    {
        camera_modules.emplace_back(
            std::make_unique<camera_driver::LuxonisCamera>(
                dev, dai::CameraBoardSocket::CAM_A, dai::ColorCameraProperties::SensorResolution::THE_720_P, width, height, frame_rate, std::to_string(idx++), running));
    }

    std::vector<std::thread> threads;
    for (auto &cm : camera_modules)
    {
        threads.emplace_back([&]()
                             { cm->Run(frame_rate); });
    }

    for (auto &t : threads)
    {
        if (t.joinable())
        {
            t.join();
        }
    }

    camera_manager.stop();
    return 0;
}
