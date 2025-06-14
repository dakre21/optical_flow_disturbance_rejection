/*
Author: David Akre
Date: 2/17/24
Description: Luxonis Camera interface implementation to depthai-core library
*/

#include "luxonis_camera.h"

namespace camera_driver {
LuxonisCamera::LuxonisCamera(
    const dai::DeviceInfo &device_info, const dai::CameraBoardSocket &socket,
    const dai::ColorCameraProperties::SensorResolution &resolution,
    const int &width, const int &height, const int &frame_rate,
    const std::string &id, std::atomic<bool> &running)
    : CameraBase(width, height, id), running_(running) {
  camera_ = pipeline_.create<dai::node::ColorCamera>();
  video_ = pipeline_.create<dai::node::XLinkOut>();

  video_->setStreamName(id_);

  camera_->setBoardSocket(socket);
  camera_->setResolution(resolution);
  camera_->setVideoSize(width, height);
  camera_->setFps(frame_rate);

  // (dakre) TODO this may require more fine-tuning, the luxonis ISP
  // is doing more in different lighting scenarios which throws off
  // the optical flow
  camera_->initialControl.setAutoFocusMode(
      dai::CameraControl::AutoFocusMode::OFF);
  camera_->initialControl.setManualExposure(1500, 800);
  camera_->initialControl.setAutoWhiteBalanceMode(
      dai::CameraControl::AutoWhiteBalanceMode::AUTO);
  camera_->initialControl.setManualFocus(255);
  // camera_->setIspScale(1,2);

  video_->input.setBlocking(false);
  video_->input.setQueueSize(10);

  camera_->video.link(video_->input);

  device_ = std::make_shared<dai::Device>(pipeline_, device_info,
                                          dai::UsbSpeed::SUPER_PLUS);
  std::cout << id_
            << "- Created pipeline for luxonis device: " << device_->getMxId()
            << std::endl;
}

void LuxonisCamera::Run(const int &frame_rate) {
  assert(device_ != nullptr);
  auto data_output_queue = device_->getOutputQueue(id_);

  std::chrono::steady_clock::time_point last_time;

  const std::chrono::milliseconds frame_duration(1000 / (2 * frame_rate));
  while (running_) {
    auto now = std::chrono::steady_clock::now();

    try {
      auto video_in = data_output_queue->get<dai::ImgFrame>();
      auto bgr = video_in->getCvFrame();

      cv::Mat gray;
      cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
      CalculateOpticalFlow(std::move(gray));
    } catch (std::runtime_error &e) {
      const auto error_msg = std::string(e.what());
      std::cerr << id_ << " " << error_msg << std::endl;
      std::cerr << "Exiting out of the loop" << std::endl;
      break;
    }

#if 0
    if (last_time.time_since_epoch().count() != 0) {
      const auto frame_time =
          std::chrono::duration<double>(now - last_time).count();
      const auto fps = 1 / frame_time;
      std::cout << id_ << "- Estimated FPS: " << std::to_string(fps)
                << std::endl;
    }
    last_time = now;
#endif

    std::this_thread::sleep_for(frame_duration);
  }
  std::cout << id_ << "- returning from thread execution" << std::endl;
}

}  // end of namespace camera_driver
