/*
Author: David Akre
Date: 2/17/24
Description: CameraModule3 interface implementation to libcamera driver
*/

#include <sys/mman.h>
#include <chrono>
#include <thread>

#include "camera_module3.h"

namespace camera_driver
{

    CameraModule3::CameraModule3(std::shared_ptr<libcamera::Camera> camera, const int &width, const int &height, const std::string &id, std::atomic<bool> &running) : camera_(camera), id_(id), running_(running)
    {
        camera_->acquire();

        config_ = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
        auto &stream_config = config_->at(0);

        stream_config.size.width = width;
        stream_config.size.height = height;
        stream_config.pixelFormat = libcamera::formats::YUV420;
        config_->validate();

        std::cout << "Stream config: " << stream_config.toString() << std::endl;
        camera_->configure(config_.get());
    }

    CameraModule3::~CameraModule3()
    {
        Stop();
    }

    void CameraModule3::Stop()
    {
        camera_->stop();
        camera_->release();
    }

    void CameraModule3::RequestComplete(libcamera::Request *request)
    {
        if (request->status() == libcamera::Request::RequestCancelled)
        {
            std::cerr << "Request cancelled in frame aquisition" << std::endl;
            return;
        }
        auto now = std::chrono::steady_clock::now();
        if (last_time_.time_since_epoch().count() != 0)
        {
            const auto frame_time = std::chrono::duration<double>(now - last_time_).count();
            const auto fps = 1 / frame_time;
            std::cout << id_ << "- Estimated FPS: " << std::to_string(fps) << std::endl;
        }
        last_time_ = now;

        for (auto [stream, buffer] : request->buffers())
        {
            void *memory = mmap(nullptr, buffer->planes()[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, buffer->planes()[0].fd.get(), 0);
            if (memory == MAP_FAILED)
            {
                std::cerr << "Failed to map memory" << std::endl;
                return;
            }
            const int w = stream->configuration().size.width;
            const int h = stream->configuration().size.height;
            cv::Mat yuv(h + h / 2, w, CV_8UC1, memory);
            cv::Mat gray;
            cv::cvtColor(yuv, gray, cv::COLOR_YUV2GRAY_I420);

            if (munmap(memory, buffer->planes()[0].length) == -1)
            {
                std::cerr << "munmap failed" << std::endl;
                return;
            }
        }
    }

    void CameraModule3::Run(const int &frame_rate)
    {
        libcamera::FrameBufferAllocator allocator(camera_);
        auto stream = config_->at(0).stream();
        if (allocator.allocate(stream) < 0)
        {
            throw std::runtime_error("Failed to allocate frame buffer");
        }
        std::vector<std::unique_ptr<libcamera::Request>> requests;
        for (auto &buffer : allocator.buffers(stream))
        {
            auto request = camera_->createRequest();
            if (!request)
            {
                throw std::runtime_error("Failed to create camera requests");
            }
            request->addBuffer(stream, buffer.get());
            requests.push_back(std::move(request));
        }

        camera_->requestCompleted.connect(this, &CameraModule3::RequestComplete);

        libcamera::ControlList controls(camera_->controls());
        int64_t frame_time = 1000000 / frame_rate;
        controls.set(libcamera::controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({frame_time, frame_time}));

        if (camera_->start(&controls))
        {
            throw std::runtime_error("Failed to start the camera");
        }

        auto issue_requests = [&](bool check)
        {
            for (auto &request : requests)
            {
                if (check && request->hasPendingBuffers())
                {
                    continue;
                }
                request->reuse(libcamera::Request::ReuseBuffers);
                camera_->queueRequest(request.get());
            }
        };

        const std::chrono::milliseconds frame_duration(1000 / frame_rate);
        issue_requests(false);
        while (running_)
        {
            std::this_thread::sleep_for(frame_duration);
            issue_requests(true);
        }
        std::cout << id_ << "- returning from thread execution" << std::endl;
        Stop();
    }

} // end of namespace camera_driver