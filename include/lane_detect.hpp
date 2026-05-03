#pragma once
#include <opencv2/opencv.hpp>
#include <deque>

struct LaneFit {
    cv::Vec3d left;
    cv::Vec3d right;
    bool valid = false;
};

struct LaneState {
    bool hasPrev = false;
    cv::Vec3d leftPrev;
    cv::Vec3d rightPrev;

    double smoothingAlpha = 0.25;
};

LaneFit detectLane(const cv::Mat& warpedBinary, LaneState& state,
                   int nwindows = 9, int margin = 100, int minpix = 50);

double computeCurvature(const cv::Vec3d& fit, double yEval);
double computeVehicleOffset(const cv::Vec3d& left,
                            const cv::Vec3d& right,
                            int imageWidth,
                            int imageHeight);

