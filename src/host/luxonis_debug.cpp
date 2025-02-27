/*
Author: David Akre
Date: 2/17/24
Description: Simple exe for debuging luxonis cameras
*/

#include <chrono>
#include <depthai/depthai.hpp>
#include <iostream>

int main() {
  dai::Pipeline pipeline;

  // Define source and output
  auto camRgb = pipeline.create<dai::node::ColorCamera>();
  auto xoutVideo = pipeline.create<dai::node::XLinkOut>();

  xoutVideo->setStreamName("video");

  // Properties
  camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
  // OV9282 has 800P 720P and 400P
  camRgb->setResolution(
      dai::ColorCameraProperties::SensorResolution::THE_720_P);
  // camRgb->setVideoSize(1280, 800);
  camRgb->setFps(140);

  xoutVideo->input.setBlocking(false);
  xoutVideo->input.setQueueSize(10);

  // Linking
  camRgb->video.link(xoutVideo->input);

  // Connect to device and start pipeline
  dai::Device device(pipeline);

  auto video = device.getOutputQueue("video");

  std::chrono::steady_clock::time_point last_time;
  while (true) {
    auto now = std::chrono::steady_clock::now();
    auto videoIn = video->get<dai::ImgFrame>();

    // Get BGR frame from NV12 encoded video frame to show with opencv
    // Visualizing the frame on slower hosts might have overhead
    // cv::imwrite("image.png", videoIn->getCvFrame());
    if (last_time.time_since_epoch().count() != 0) {
      const auto frame_time =
          std::chrono::duration<double>(now - last_time).count();
      const auto fps = 1 / frame_time;
      std::cout << "Estimated FPS: " << std::to_string(fps) << std::endl;
    }
    last_time = now;
  }
  return 0;
}
