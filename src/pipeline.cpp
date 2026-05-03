#include "pipeline.hpp"
#include "threshold.hpp"
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

static double evalPoly(const cv::Vec3d& c, double y) {
    return c[0]*y*y + c[1]*y + c[2];
}

// refit x(y) in meter space and compute curvature at bottom of image (in meters)
static double curvatureMeters(const cv::Vec3d& fitPix,
                              int imgH,
                              double xm_per_pix,
                              double ym_per_pix) {
    // sample and refit: x_m = A*y_m^2 + B*y_m + C
    std::vector<double> ys_m, xs_m;
    ys_m.reserve(imgH / 10 + 1);
    xs_m.reserve(imgH / 10 + 1);

    for (int y = 0; y < imgH; y += 10) {
        double x_pix = evalPoly(fitPix, (double)y);
        ys_m.push_back(y * ym_per_pix);
        xs_m.push_back(x_pix * xm_per_pix);
    }

    cv::Mat A((int)ys_m.size(), 3, CV_64F);
    cv::Mat b((int)ys_m.size(), 1, CV_64F);

    for (int i = 0; i < (int)ys_m.size(); i++) {
        double y = ys_m[i];
        A.at<double>(i, 0) = y * y;
        A.at<double>(i, 1) = y;
        A.at<double>(i, 2) = 1.0;
        b.at<double>(i, 0) = xs_m[i];
    }

    cv::Mat x;
    if (!cv::solve(A, b, x, cv::DECOMP_SVD)) return 1e12;

    double A_m = x.at<double>(0,0);
    double B_m = x.at<double>(1,0);

    double y_eval_m = (imgH - 1) * ym_per_pix;
    if (std::abs(A_m) < 1e-12) return 1e12;

    double term = 2.0 * A_m * y_eval_m + B_m;
    return std::pow(1.0 + term * term, 1.5) / std::abs(2.0 * A_m);
}

cv::Mat processFrame(const cv::Mat& frame, PipelineState& state) {
    auto evalPoly = [](const cv::Vec3d& c, double y) -> double {
        return c[0]*y*y + c[1]*y + c[2];
    };

    // undistort
    cv::Mat undist = undistortImage(frame, state.calib);

    // threshold to binary (0/255)
    cv::Mat binary = thresholdBinary(undist);

    // warp to birds-eye (binary)
    cv::Mat warped = warpPerspectiveBinary(binary, state.persp);

    // detect lane
    LaneFit fit = detectLane(warped, state.laneState, 9, 100, 50);

    // prepare outputs for saving (only valid if fit.valid)
    cv::Mat laneWarp, laneUnwarped;

    // compose final frame
    cv::Mat out = undist.clone();

    if (!fit.valid) {
        cv::putText(out, "Lane detection failed", {40,40},
                    cv::FONT_HERSHEY_SIMPLEX, 1, {0,0,255}, 2);
    } else {
        int H = warped.rows;
        int W = warped.cols;

        // build lane polygon in warped space
        std::vector<cv::Point> pts;
        pts.reserve(2 * H);

        for (int y = 0; y < H; y++) {
            double xL = evalPoly(fit.left, (double)y);
            pts.emplace_back((int)std::round(xL), y);
        }
        for (int y = H - 1; y >= 0; y--) {
            double xR = evalPoly(fit.right, (double)y);
            pts.emplace_back((int)std::round(xR), y);
        }

        laneWarp = cv::Mat::zeros(H, W, CV_8UC3);
        std::vector<std::vector<cv::Point>> poly{pts};
        cv::fillPoly(laneWarp, poly, cv::Scalar(0, 255, 0));

        // unwarp lane overlay back to undistorted space
        cv::warpPerspective(laneWarp, laneUnwarped, state.persp.Minv, undist.size());

        // overlay on output
        cv::addWeighted(out, 1.0, laneUnwarped, 0.3, 0.0, out);

        // curvature + offset (meters)
        double yEval = (double)(H - 1);
        double leftX = evalPoly(fit.left, yEval);
        double rightX = evalPoly(fit.right, yEval);

        double laneWidthPx = std::max(1.0, rightX - leftX);
        double xm_per_pix = 3.7 / laneWidthPx;        
        double ym_per_pix = 30.0 / (double)H;         

        double curvL_m = curvatureMeters(fit.left, H, xm_per_pix, ym_per_pix);
        double curvR_m = curvatureMeters(fit.right, H, xm_per_pix, ym_per_pix);
        double curv_m  = 0.5 * (curvL_m + curvR_m);

        double offset_m = computeVehicleOffset(fit.left, fit.right, out.cols, out.rows);

        cv::putText(out, "Curvature: " + std::to_string((int)curv_m) + " m",
                    {40,40}, cv::FONT_HERSHEY_SIMPLEX, 1, {0,255,0}, 2);
        cv::putText(out, "Offset: " + std::to_string(offset_m) + " m",
                    {40,80}, cv::FONT_HERSHEY_SIMPLEX, 1, {0,255,0}, 2);
    }

    // save intermediates per-image (only if output_dir is set)
    if (!state.output_dir.empty()) {
        std::filesystem::create_directories(state.output_dir);

        cv::imwrite(state.output_dir + "/undistorted.jpg", undist);
        cv::imwrite(state.output_dir + "/binary.jpg", binary);
        cv::imwrite(state.output_dir + "/warped_binary.jpg", warped);

        if (fit.valid) {
            cv::imwrite(state.output_dir + "/lane_warp.jpg", laneWarp);
            cv::imwrite(state.output_dir + "/lane_unwarped.jpg", laneUnwarped);
        }

        cv::imwrite(state.output_dir + "/result.jpg", out);
    }

    return out;
}
