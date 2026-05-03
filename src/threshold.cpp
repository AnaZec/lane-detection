#include "threshold.hpp"
#include <opencv2/opencv.hpp>

static cv::Mat applyCLAHEToLightness(const cv::Mat& bgr) {
    cv::Mat hls;
    cv::cvtColor(bgr, hls, cv::COLOR_BGR2HLS);

    std::vector<cv::Mat> channels;
    cv::split(hls, channels);

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
        2.0,            // clip limit
        cv::Size(8, 8)  // tile grid size
    );

    clahe->apply(channels[1], channels[1]); // HLS lightness channel

    cv::merge(channels, hls);

    cv::Mat enhanced;
    cv::cvtColor(hls, enhanced, cv::COLOR_HLS2BGR);

    return enhanced;
}

static cv::Mat sobelXBinary(const cv::Mat& gray, int ksize, int threshMin, int threshMax) {
    cv::Mat gradX, absGradX;
    cv::Sobel(gray, gradX, CV_32F, 1, 0, ksize);
    cv::convertScaleAbs(gradX, absGradX); // 8-bit abs

    cv::Mat binary;
    cv::inRange(absGradX, threshMin, threshMax, binary);
    return binary; // 0 or 255
}

static cv::Mat whiteMask(const cv::Mat& bgr) {
    cv::Mat hls;
    cv::cvtColor(bgr, hls, cv::COLOR_BGR2HLS);
    std::vector<cv::Mat> ch;
    cv::split(hls, ch);
    cv::Mat L = ch[1];
    cv::Mat S = ch[2];

    cv::Mat maskL, maskS, mask;
    cv::inRange(L, 200, 255, maskL);   // bright
    cv::inRange(S, 0, 110, maskS);     // not too saturated
    cv::bitwise_and(maskL, maskS, mask);
    return mask;
}

static cv::Mat yellowMask(const cv::Mat& bgr) {
    // yellow pixels: HSV range
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(15, 80, 80), cv::Scalar(35, 255, 255), mask);
    return mask;
}

cv::Mat thresholdBinary(const cv::Mat& undistorted) {
    // normalize local illumination before fixed color/gradient thresholds -> improves lane visibility under shadows and uneven lighting
    cv::Mat normalized = applyCLAHEToLightness(undistorted);

    // color masks
    cv::Mat maskW = whiteMask(normalized);
    cv::Mat maskY = yellowMask(normalized);
    cv::Mat colorMask;
    cv::bitwise_or(maskW, maskY, colorMask);

    // gradient mask (edges)
    cv::Mat gray;
    cv::cvtColor(normalized, gray, cv::COLOR_BGR2GRAY);
    cv::Mat gradMask = sobelXBinary(gray, 3, 30, 255);

    // combine: (color OR gradient)
    cv::Mat combined;
    cv::bitwise_or(colorMask, gradMask, combined);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(combined, combined, cv::MORPH_CLOSE, kernel);

    return combined; // 0 or 255
}