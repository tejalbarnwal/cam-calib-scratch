#include <iostream>
#include <utility>
#include <string>

#include <eigen3/Eigen/Dense>

#include "../include/calibration.h"
#include <vector>


int main(int argc, char const *argv[])
{
    std::cout << "\n-- Calibration Source --\n";

    std::string abs_path_img_folder = "bol";
    std::pair<int, int> pattern_size = std::make_pair(7, 6);
    double square_size = 1.0;
    bool show_corners_on_img = true;

    // std::unique_ptr<Eigen::MatrixXd> X_matrix;
    // std::unique_ptr<Eigen::MatrixXd> x_matrix;

    Eigen::MatrixXd X_matrix;
    Eigen::MatrixXd x_matrix;

    std::vector<std::vector<cv::Point2f>> img_points;
    std::vector<std::vector<cv::Point2f>> object_points;

    calibration::findChessBoardCorner(abs_path_img_folder,
                                    pattern_size,
                                    square_size,
                                    show_corners_on_img,
                                    object_points, img_points);

    // make variables for homography matrix, intrinsic matrix, extrinsic matrix, lens distortion
    // Eigen::MatrixXd Hn_matrix;
    std::vector<Eigen::Matrix3d> Hn;
    Eigen::Matrix3d K;
    std::vector<Eigen::Matrix3d> R;
    std::vector<Eigen::Vector3d> t;
    Eigen::Vector4d d;
    calibration::calibrate(object_points, img_points, Hn, K, R, t, d);

    return 0;


}