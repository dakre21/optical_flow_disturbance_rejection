/*
Author: David Akre
Date: 2/17/24
Description: CameraModule3 interface to libcamera driver
*/
#pragma once

#include <atomic>

#include <libcamera/libcamera.h>
#include <libcamera/control_ids.h>

#include "camera_base.h"

namespace camera_driver
{
    class CameraModule3 : public CameraBase
    {
    public:
        CameraModule3(std::shared_ptr<libcamera::Camera> camera,
                      const int &width, const int &height, const std::string &id, std::atomic<bool> &running);

        ~CameraModule3();

        void Stop();

        void Run(const int &frame_rate) final;

    private:
        void RequestComplete(libcamera::Request *request);

        std::chrono::steady_clock::time_point last_time_;
        std::shared_ptr<libcamera::Camera> camera_;
        std::unique_ptr<libcamera::CameraConfiguration> config_;
        std::string id_;
        std::atomic<bool> &running_;
    };
} // end of namespace camera_driver