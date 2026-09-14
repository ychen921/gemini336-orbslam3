#pragma once

#include <opencv2/core.hpp>

namespace gemini336_orbslam3
{
struct StereoFrame
{
    cv::Mat left;
    cv::Mat right;
    double timestamp = 0.0;
};
}
