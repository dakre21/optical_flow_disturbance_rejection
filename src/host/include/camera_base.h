/*
Author: David Akre
Date: 2/17/24
Description: Base camera class to run optical flow on each frame
*/
#pragma once

#include <boost/interprocess/exceptions.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

namespace bip = boost::interprocess;

// (dakre) TODO may need to play around with this during testing
#define NUM_POINTS 100
#define FLOW_THRESHOLD 5

namespace camera_driver {
class CameraBase {
 public:
  CameraBase(const int &width, const int &height, const std::string &id)
      : id_(id) {
    assert(width == 640 && height == 480);
    auto linspace = [&](auto start, auto end, auto num) {
      assert(num > 0);

      std::vector<double> result(num);
      auto step = (end - start) / num;
      for (auto i = 0; i < num; ++i) {
        result[i] = start + i * step;
      }
      return result;
    };

    const float ar = (float)width / (float)height;
    const int num_x = 10 * ar;
    const int num_y = 10 / ar;
    assert(num_x * num_y <= NUM_POINTS);

    const auto x_points = linspace(0, width - 1, num_x);
    const auto y_points = linspace(0, height - 1, num_y);

    feature_points_.resize(NUM_POINTS);

    int idx = 0;
    for (const auto &y : y_points) {
      for (const auto &x : x_points) {
        feature_points_[idx++] = cv::Point2f(x, y);
      }
    }

    const auto win_size = cv::Size(15, 15);
    const auto criteria = cv::TermCriteria(
        (cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 1, 0.1);
    const auto num_pyramids = 1;
    lk_ = cv::SparsePyrLKOpticalFlow::create(win_size, num_pyramids, criteria);
    // lk_->setFlags(cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

    const auto shm_name = std::string("optical_flow_vectors_") + id_;
    bip::shared_memory_object::remove(shm_name.c_str());

    shm_ = std::make_unique<bip::shared_memory_object>(
        bip::create_only, shm_name.c_str(), bip::read_write);

    const auto shm_size = sizeof(float) * feature_points_.size() * 2;
    shm_->truncate(shm_size);

    shm_region_ = std::make_unique<bip::mapped_region>(*shm_, bip::read_write);

  }

  virtual ~CameraBase() {}

  virtual void Run(const int &frame_rate) = 0;

  virtual void CalculateOpticalFlow(cv::Mat &&gray) {
    assert(lk_ != nullptr);
    assert(shm_region_ != nullptr);

    std::vector<float> u(feature_points_.size());
    std::vector<float> v(feature_points_.size());
    if (last_frame_ != nullptr) {
      std::vector<unsigned char> status;
      std::vector<cv::Point2f> new_points;

#if 0
      cv::Mat diff;
      cv::absdiff(*last_frame_, gray, diff);

      int threshold_value = 10;
      cv::Mat mask = diff < threshold_value;
      int count = cv::countNonZero(mask);
      std::cout << "Number of pixels with value < " << threshold_value << " : "
                << count << std::endl;
      cv::imshow("Image", diff);
      cv::waitKey(0);
#endif

      cv::GaussianBlur(*last_frame_, *last_frame_, cv::Size(5, 5), 0);
      cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

      lk_->calc(*last_frame_, gray, feature_points_, new_points, status);

      for (size_t i = 0; i < feature_points_.size(); ++i) {
        float u_pt = 0, v_pt = 0;
        if (1 == status[i]) {
          u_pt = feature_points_[i].x - new_points[i].x;
          v_pt = feature_points_[i].y - new_points[i].y;
          if (std::hypot(u_pt, v_pt) >= FLOW_THRESHOLD) {
            u_pt = 0;
            v_pt = 0;
          }
        }
        u[i] = u_pt;
        v[i] = v_pt;
        // std::cout << u[i] << ", " << v[i] << std::endl;
      }

      const size_t num_bytes = feature_points_.size() * sizeof(float);
      std::memcpy(shm_region_->get_address(), u.data(), num_bytes);
      std::memcpy(
          static_cast<std::byte *>(shm_region_->get_address()) + num_bytes,
          v.data(), num_bytes);
    }

    last_frame_ = std::make_unique<cv::Mat>(std::move(gray));
  }

 protected:
  std::string id_;

 private:
  std::vector<cv::Point2f> feature_points_;
  cv::Ptr<cv::SparsePyrLKOpticalFlow> lk_;
  std::unique_ptr<cv::Mat> last_frame_;
  std::unique_ptr<bip::shared_memory_object> shm_;
  std::unique_ptr<bip::mapped_region> shm_region_;
};
}  // end of namespace camera_driver
