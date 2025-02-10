#ifndef VISION_UTILS_HPP
#define VISION_UTILS_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

// Define floating point precision type
using FPTYPE = double;

// Eigen types
using Matrix_t = Eigen::Matrix<FPTYPE, 3, 3>;
using Vector3D_t = Eigen::Matrix<FPTYPE, 3, 1>;
using Point2D_t = Eigen::Matrix<FPTYPE, 2, 1>;
using Point3D_t = Eigen::Matrix<FPTYPE, 3, 1>;

// Homography solution types
enum class homographySolution { SOLUTION_1, SOLUTION_2 };

namespace vision_utils {

Matrix_t ComputeHomography(const std::vector<cv::Point2d>& cv_reference_points,
                           const std::vector<cv::Point2d>& cv_current_points,
                           const Matrix_t& K);

void RecoverFromHomography(const Matrix_t& homography,
                           Matrix_t& R,
                           Vector3D_t& t,
                           Vector3D_t& n,
                           FPTYPE& distancePlane,
                           int current_iteration,
                           homographySolution& homography_solution);

void EstimateHomography(const cv::Mat& refImg,
                        const cv::Mat& currImg,
                        const Matrix_t& K,
                        Matrix_t& H,
                        int counter);

void Rodriguez(const Matrix_t& R, Vector3D_t& u);

void PBVSController(const Matrix_t& R,
                    const Vector3D_t& t,
                    const Vector3D_t& u,
                    Vector3D_t& Uv,
                    Vector3D_t& Uw,
                    const FPTYPE lambdav_,
                    const FPTYPE lambdaw_);

Point2D_t NormalizePoint(const Point2D_t& point, const Matrix_t& Kinv_);

} // namespace vision_utils

#endif // VISION_UTILS_HPP

