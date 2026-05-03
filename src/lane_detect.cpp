#include "lane_detect.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>

static bool polyfit_x_of_y(const std::vector<double>& ys,
                           const std::vector<double>& xs,
                           cv::Vec3d& coeffs_out) {
    if (ys.size() < 50 || xs.size() != ys.size()) return false;

    // Fit x = A*y^2 + B*y + C
    cv::Mat A((int)ys.size(), 3, CV_64F);
    cv::Mat b((int)ys.size(), 1, CV_64F);

    for (int i = 0; i < (int)ys.size(); i++) {
        double y = ys[i];
        A.at<double>(i, 0) = y * y;
        A.at<double>(i, 1) = y;
        A.at<double>(i, 2) = 1.0;
        b.at<double>(i, 0) = xs[i];
    }

    cv::Mat x;
    bool ok = cv::solve(A, b, x, cv::DECOMP_SVD);
    if (!ok) return false;

    coeffs_out = cv::Vec3d(x.at<double>(0,0), x.at<double>(1,0), x.at<double>(2,0));
    return true;
}

LaneFit detectLane(const cv::Mat& warpedBinary, LaneState& state,
                   int nwindows, int margin, int minpix) {
    LaneFit fit;

    // expect binary 0/255
    CV_Assert(warpedBinary.type() == CV_8UC1);

    // histogram over bottom half
    cv::Mat bottomHalf = warpedBinary.rowRange(warpedBinary.rows/2, warpedBinary.rows);
    cv::Mat histogram;
    cv::reduce(bottomHalf, histogram, 0, cv::REDUCE_SUM, CV_32S);

    cv::Mat hist64;
    histogram.convertTo(hist64, CV_64F);

    int mid = hist64.cols / 2;

    // peaks
    double minValL, maxValL, minValR, maxValR;
    cv::Point minLocL, maxLocL, minLocR, maxLocR;

    cv::minMaxLoc(hist64.colRange(0, mid), &minValL, &maxValL, &minLocL, &maxLocL);
    cv::minMaxLoc(hist64.colRange(mid, hist64.cols), &minValR, &maxValR, &minLocR, &maxLocR);

    int leftx_base = maxLocL.x;
    int rightx_base = maxLocR.x + mid;

    // find nonzero pixels once
    std::vector<cv::Point> nonzero;
    cv::findNonZero(warpedBinary, nonzero);

    int window_height = warpedBinary.rows / nwindows;

    int leftx_current = leftx_base;
    int rightx_current = rightx_base;

    std::vector<double> leftx, lefty, rightx, righty;
    leftx.reserve(5000); lefty.reserve(5000);
    rightx.reserve(5000); righty.reserve(5000);

    for (int win = 0; win < nwindows; win++) {
        int win_y_low  = warpedBinary.rows - (win + 1) * window_height;
        int win_y_high = warpedBinary.rows - win * window_height;

        int win_xleft_low  = leftx_current  - margin;
        int win_xleft_high = leftx_current  + margin;

        int win_xright_low  = rightx_current - margin;
        int win_xright_high = rightx_current + margin;

        int left_count = 0;
        int right_count = 0;
        double left_sumx = 0.0;
        double right_sumx = 0.0;

        for (const auto& p : nonzero) {
            int x = p.x;
            int y = p.y;

            if (y >= win_y_low && y < win_y_high) {
                if (x >= win_xleft_low && x < win_xleft_high) {
                    leftx.push_back((double)x);
                    lefty.push_back((double)y);
                    left_count++;
                    left_sumx += x;
                }
                if (x >= win_xright_low && x < win_xright_high) {
                    rightx.push_back((double)x);
                    righty.push_back((double)y);
                    right_count++;
                    right_sumx += x;
                }
            }
        }

        // recenter if enough pixels found
        if (left_count > minpix)  leftx_current  = (int)std::round(left_sumx  / left_count);
        if (right_count > minpix) rightx_current = (int)std::round(right_sumx / right_count);
    }

    cv::Vec3d leftFit, rightFit;
    bool okL = polyfit_x_of_y(lefty, leftx, leftFit);
    bool okR = polyfit_x_of_y(righty, rightx, rightFit);

    if (!(okL && okR)) {
        fit.valid = false;
        return fit;
    }

    // Lane geometry sanity checks.
    // The detector should reject polynomial fits that are geometrically implausible
    // instead of blindly accepting every sliding-window result
    const double yTop = 0.0;
    const double yMid = warpedBinary.rows * 0.5;
    const double yBottom = warpedBinary.rows - 1.0;

    auto evalX = [](const cv::Vec3d& c, double y) {
        return c[0] * y * y + c[1] * y + c[2];
    };

    const double leftTop = evalX(leftFit, yTop);
    const double rightTop = evalX(rightFit, yTop);
    const double leftMid = evalX(leftFit, yMid);
    const double rightMid = evalX(rightFit, yMid);
    const double leftBottom = evalX(leftFit, yBottom);
    const double rightBottom = evalX(rightFit, yBottom);

    const double widthTop = rightTop - leftTop;
    const double widthMid = rightMid - leftMid;
    const double widthBottom = rightBottom - leftBottom;

    constexpr double minLaneWidthPx = 350.0;
    constexpr double maxLaneWidthPx = 900.0;
    constexpr double maxWidthVariationPx = 250.0;
    constexpr double maxCurvatureCoeffDiff = 0.001;

    const bool validOrdering =
        widthTop > 0.0 &&
        widthMid > 0.0 &&
        widthBottom > 0.0;

    const bool validWidth =
        widthBottom >= minLaneWidthPx &&
        widthBottom <= maxLaneWidthPx &&
        widthMid >= minLaneWidthPx &&
        widthMid <= maxLaneWidthPx;

    const bool stableWidth =
        std::abs(widthBottom - widthMid) <= maxWidthVariationPx &&
        std::abs(widthMid - widthTop) <= maxWidthVariationPx;

    const bool similarCurvature =
        std::abs(leftFit[0] - rightFit[0]) <= maxCurvatureCoeffDiff;

    if (!(validOrdering && validWidth && stableWidth && similarCurvature)) {
        fit.valid = false;
        return fit;
    }

    cv::Vec3d smoothedLeft = leftFit;
    cv::Vec3d smoothedRight = rightFit;

    if (state.hasPrev) {
        const double alpha = state.smoothingAlpha;

        smoothedLeft = alpha * leftFit + (1.0 - alpha) * state.leftPrev;
        smoothedRight = alpha * rightFit + (1.0 - alpha) * state.rightPrev;
    }

    fit.left = smoothedLeft;
    fit.right = smoothedRight;
    fit.valid = true;

    state.leftPrev = smoothedLeft;
    state.rightPrev = smoothedRight;
    state.hasPrev = true;

    return fit;
}

double computeCurvature(const cv::Vec3d& fit, double yEval) {
    double A = fit[0];
    double B = fit[1];
    if (std::abs(A) < 1e-12) return 1e12;
    double term = 2.0*A*yEval + B;
    return std::pow(1.0 + term*term, 1.5) / std::abs(2.0*A);
}

double computeVehicleOffset(const cv::Vec3d& left,
                            const cv::Vec3d& right,
                            int imageWidth,
                            int imageHeight) {
    double y = (double)(imageHeight - 1);

    auto evalX = [&](const cv::Vec3d& c) {
        return c[0]*y*y + c[1]*y + c[2];
    };

    double leftX = evalX(left);
    double rightX = evalX(right);
    double laneCenter = (leftX + rightX) / 2.0;
    double vehicleCenter = imageWidth / 2.0;

    double laneWidthPx = std::max(1.0, rightX - leftX);
    double xm_per_pix = 3.7 / laneWidthPx;

    return (vehicleCenter - laneCenter) * xm_per_pix;
}
