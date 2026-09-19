#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace gemini336_orbslam3
{
struct StereoFrame
{
    // Reference-counted MONO8 pixels; copying the frame retains the image storage.
    cv::Mat left;
    cv::Mat right;

    // Left image acquisition time in seconds from its ROS message header.
    double timestamp = 0.0;
};

struct ImuMeasurement
{
    // Seconds from the IMU message header stamp, on the same time basis as StereoFrame.
    double timestamp = 0.0;

    // Preserve the source IMU frame; no rotation or gravity removal is applied.
    Eigen::Vector3f accel = Eigen::Vector3f::Zero();  // m/s^2
    Eigen::Vector3f gyro = Eigen::Vector3f::Zero();   // rad/s
};
}
