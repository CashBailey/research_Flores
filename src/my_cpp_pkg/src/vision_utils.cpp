
#include "my_cpp_pkg/vision_utils.hpp"
#include <opencv2/calib3d.hpp>

namespace vision_utils {

Matrix_t ComputeHomography(const std::vector<cv::Point2d>& cv_reference_points,
                           const std::vector<cv::Point2d>& cv_current_points,
                           const Matrix_t& K) {
    std::vector<cv::Point2f> ref_pts, curr_pts;
    for (const auto& pt : cv_reference_points) {
        ref_pts.emplace_back(static_cast<float>(pt.x), static_cast<float>(pt.y));
    }
    for (const auto& pt : cv_current_points) {
        curr_pts.emplace_back(static_cast<float>(pt.x), static_cast<float>(pt.y));
    }

    cv::Mat H_cv = cv::findHomography(ref_pts, curr_pts, cv::RANSAC);
    Matrix_t H_eigen;
    cv::cv2eigen(H_cv, H_eigen);
    return K.inverse() * H_eigen * K;
}

void RecoverFromHomography(const Matrix_t& homography,
                           Matrix_t& R,
                           Vector3D_t& t,
                           Vector3D_t& n,
                           FPTYPE& distancePlane,
                           int current_iteration,
                           homographySolution& homography_solution) {
    Eigen::JacobiSVD<Matrix_t> svd(homography, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d singular_values = svd.singularValues();

    const Matrix_t& U = svd.matrixU();
    const Matrix_t& V = svd.matrixV();

    Matrix_t W;
    W << 1, 0, 0,
         0, 1, 0,
         0, 0, singular_values[0]/singular_values[1];

    Matrix_t R1 = U * W * V.transpose();
    Matrix_t R2 = U * W.transpose() * V.transpose();

    std::vector<Matrix_t> possible_rotations = { R1, R2 };
    std::vector<Vector3D_t> possible_translations, possible_normals;

    for (const auto& R_candidate : possible_rotations) {
        Vector3D_t t_candidate = U.col(2) * (singular_values[0] + singular_values[1]) / 2.0;
        Vector3D_t n_candidate = V.col(2).normalized();

        possible_translations.push_back(t_candidate);
        possible_normals.push_back(n_candidate);
    }

    // Simplified selection: choose first solution for demonstration
    R = possible_rotations[0];
    t = possible_translations[0];
    n = possible_normals[0];
    distancePlane = 1.0; // Placeholder
    homography_solution = homographySolution::SOLUTION_1;
}

void EstimateHomography(const cv::Mat& refImg,
                        const cv::Mat& currImg,
                        const Matrix_t& K,
                        Matrix_t& H,
                        int counter) {
    cv::Ptr<cv::ORB> orb = cv::ORB::create(500);
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat des1, des2;
    orb->detectAndCompute(refImg, cv::noArray(), kp1, des1);
    orb->detectAndCompute(currImg, cv::noArray(), kp2, des2);

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher.match(des1, des2, matches);

    std::vector<cv::Point2d> ref_pts, curr_pts;
    for (const auto& m : matches) {
        ref_pts.push_back(kp1[m.queryIdx].pt);
        curr_pts.push_back(kp2[m.trainIdx].pt);
    }

    H = ComputeHomography(ref_pts, curr_pts, K);
}

void Rodriguez(const Matrix_t& R, Vector3D_t& u) {
    cv::Mat R_cv;
    cv::eigen2cv(R, R_cv);
    cv::Mat rvec;
    cv::Rodrigues(R_cv, rvec);
    cv::cv2eigen(rvec, u);
}

void PBVSController(const Matrix_t& R,
                    const Vector3D_t& t,
                    const Vector3D_t& u,
                    Vector3D_t& Uv,
                    Vector3D_t& Uw,
                    const FPTYPE lambdav_,
                    const FPTYPE lambdaw_) {
    Uv = -lambdav_ * R.transpose() * t;
    Uw = -lambdaw_ * u;
}

Point2D_t NormalizePoint(const Point2D_t& point, const Matrix_t& Kinv_) {
    Vector3D_t homog_point(point.x(), point.y(), 1.0);
    Vector3D_t normalized = Kinv_ * homog_point;
    return Point2D_t(normalized.x() / normalized.z(), normalized.y() / normalized.z());
}

} // namespace vision_utils
