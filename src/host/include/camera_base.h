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

namespace camera_driver {
class CameraBase {
 public:
  CameraBase(const int &width, const int &height, const std::string &id,
             const int num_per_circle = 100)
      : id_(id) {
    auto linspace = [&](auto start, auto end, auto num) {
      assert(num > 0);

      std::vector<double> result(num);
      auto step = (end - start) / num;
      for (auto i = 0; i < num; ++i) {
        result[i] = start + i * step;
      }
      return result;
    };

    const std::vector<int> center{height / 2, width / 2};
    const auto radii = linspace(0.1, 1.0, 8);
    for (const auto &r : radii) {
      auto theta = linspace(0, 2 * M_PI, num_per_circle);
      for (const auto &t : theta) {
        const auto x = center[0] + r * cos(t);
        const auto y = center[1] + r * sin(t);
        if (x >= 0 && x < width && y >= 0 && y < height) {
          feature_points_.emplace_back(cv::Point2f(x, y));
        }
      }
    }

    const auto win_size = cv::Size(7, 7);
    const auto criteria = cv::TermCriteria(
        (cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 1, 0.03);
    const auto num_pyramids = 1;
    lk_ = cv::SparsePyrLKOpticalFlow::create(win_size, num_pyramids, criteria);
    lk_->setFlags(cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

    shm_name_ = std::string("optical_flow_vectors_") + id_;
    bip::shared_memory_object::remove(shm_name_.c_str());

    shm_ = std::make_unique<bip::shared_memory_object>(
        bip::create_only, shm_name_.c_str(), bip::read_write);

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

      lk_->calc(*last_frame_, gray, feature_points_, new_points, status);

      for (size_t i = 0; i < feature_points_.size(); ++i) {
        float u_pt = 0, v_pt = 0;
        if (1 == status[i]) {
          u_pt = feature_points_[i].x - new_points[i].x;
          v_pt = feature_points_[i].x - new_points[i].y;
        }
        u[i] = u_pt;
        v[i] = v_pt;
      }

      const size_t num_bytes = feature_points_.size() * sizeof(float);
      std::memcpy(shm_region_->get_address(), u.data(), num_bytes);
      std::memcpy(shm_region_->get_address() + num_bytes, v.data(), num_bytes);
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
  std::string shm_name_;
};
}  // end of namespace camera_driver
