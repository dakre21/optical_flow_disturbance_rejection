/*
Author: David Akre
Date: 2/17/24
Description: Luxonis Camera interface to depthai-core library
*/
#pragma once

#include <atomic>
#include <depthai/depthai.hpp>
#include <memory>
#include <string>

#include "camera_base.h"

namespace camera_driver {
class LuxonisCamera : public CameraBase {
 public:
  LuxonisCamera(const dai::DeviceInfo &device_info,
                const dai::CameraBoardSocket &socket,
                const dai::ColorCameraProperties::SensorResolution &resolution,
                const int &width, const int &height, const int &frame_rate,
                const std::string &, std::atomic<bool> &running);

  void Run(const int &frame_rate) final;

 private:
  dai::Pipeline pipeline_;
  std::atomic<bool> &running_;
  std::shared_ptr<dai::Device> device_;
  std::shared_ptr<dai::node::ColorCamera> camera_;
  std::shared_ptr<dai::node::XLinkOut> video_;
};
}  // end of namespace camera_driver
