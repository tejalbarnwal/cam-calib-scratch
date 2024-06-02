#include <iostream>
#include <utility>
#include <string>

#include <eigen3/Eigen/Dense>

#include "../include/calibration.h"


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

    calibration::findChessBoardCorner(abs_path_img_folder,
                                    pattern_size,
                                    square_size,
                                    show_corners_on_img,
                                    &X_matrix, &x_matrix);

    // make variables for homography matrix, intrinsic matrix, extrinsic matrix, lens distortion
    Eigen::MatrixXd Hn_matrix;
    calibration::calibrate(&X_matrix, &x_matrix, &Hn_matrix);

    return 0;


}