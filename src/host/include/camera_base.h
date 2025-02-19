/*
Author: David Akre
Date: 2/17/24
Description: Base camera class to run optical flow on each frame
*/
#pragma once

#include <cmath>

#include "opencv2/opencv.hpp"

namespace camera_driver
{
    class CameraBase
    {
    public:
        CameraBase(const int &width, const int &height, const int num_per_circle = 100)
        {
            auto linspace = [&](auto start, auto end, auto num)
            {
                assert(num > 0);

                std::vector<double> result(num);
                auto step = (end - start) / num;
                for (auto i = 0; i < num; ++i)
                {
                    result[i] = start + i * step;
                }
                return result;
            };

            const std::vector<int> center{height / 2, width / 2};
            const auto radii = linspace(0.1, 1.0, 8);
            for (const auto &r : radii)
            {
                auto theta = linspace(0, 2 * M_PI, num_per_circle);
                for (const auto &t : theta)
                {
                    const auto x = center[0] + r * cos(t);
                    const auto y = center[1] + r * sin(t);
                    if (x >= 0 && x < width && y >= 0 && y < height)
                    {
                        feature_points_.emplace_back(cv::Point2f(x, y));
                    }
                }
            }
        }

        virtual ~CameraBase() {}

        virtual void Run(const int &frame_rate) = 0;

        virtual cv::Mat CalculateOpticalFlow(cv::Mat &&gray)
        {
            cv::Mat flow;
            if (last_frame_ != nullptr)
            {
                #if 0
                std::vector<unsigned char> status;
                std::vector<float> err;
                std::vector<cv::Point2f> new_points;
                //const auto criteria = cv::TermCriteria();
                const auto criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 1, 0.1);
                const auto win_size = cv::Size(15, 15);
                const auto num_pyramids = 1;
                cv::calcOpticalFlowPyrLK(*last_frame_,
                                         gray,
                                         feature_points_,
                                         new_points,
                                         status,
                                         err,
                                         win_size,
                                         num_pyramids,
                                         criteria);
                #endif 
                const auto w = gray.cols;
                const auto h = gray.rows;
                int64_t res = 0;
                for (int i = 0; i < w; i++) {
                    for (int j = 0; j < h; j++) {
                        res = i + j;
                    }
                }

                // TODO - filter points and return result
            }

            last_frame_ = std::make_unique<cv::Mat>(std::move(gray));

            return flow;
        }

    private:
        std::vector<cv::Point2f> feature_points_;
        std::unique_ptr<cv::Mat> last_frame_;
    };
} // end of namespace camera_driver