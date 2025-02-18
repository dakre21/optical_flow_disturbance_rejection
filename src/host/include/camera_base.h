/*
Author: David Akre
Date: 2/17/24
Description: Base camera class to run optical flow on each frame
*/
#pragma once

#include "opencv2/opencv.hpp"

namespace camera_driver
{
    struct FarnebackParams
    {
        FarnebackParams() : num_levels(3), pyr_scale(0.5), fast_pyramids(false),
                            win_size(15), num_iters(3), poly_n(5), poly_sigma(1.2), flags(0)
        {
            // noop
        }

        int num_levels;
        double pyr_scale;
        bool fast_pyramids;
        int win_size;
        int num_iters;
        int poly_n;
        double poly_sigma;
        int flags;
    };

    class CameraBase
    {
    public:
        CameraBase() {}

        virtual ~CameraBase() {}

        virtual void Run(const int &frame_rate) = 0;

        virtual cv::Mat CalculateOpticalFlow(cv::Mat &&gray)
        {
            cv::Mat flow;
            if (last_frame_ != nullptr)
            {
                FarnebackParams params;
                cv::calcOpticalFlowFarneback(*last_frame_,
                                             gray,
                                             flow,
                                             params.pyr_scale,
                                             params.num_levels,
                                             params.win_size,
                                             params.num_iters,
                                             params.poly_n,
                                             params.poly_sigma,
                                             params.flags);
            }

            last_frame_ = std::make_unique<cv::Mat>(std::move(gray));

            return flow;
        }

    private:
        std::unique_ptr<cv::Mat> last_frame_;
    };
} // end of namespace camera_driver