#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/types_c.h>

#include <eigen3/Eigen/Dense>

// #include "homography.h"


namespace calibration
{
    void findChessBoardCorner(std::string folder_path,
                                std::pair<int, int> pattern_size,
                                double square_size,
                                bool show_corners_on_img,
                                std::vector<std::vector<cv::Point2f>> &object_points,
                                std::vector<std::vector<cv::Point2f>> &img_points);

    // might pass eigen paramters
    void calibrate(std::vector<std::vector<cv::Point2f>> &object_points, 
                    std::vector<std::vector<cv::Point2f>> &img_points, 
                    std::vector<Eigen::Matrix3d> &Hn_matrix);
}


#endif