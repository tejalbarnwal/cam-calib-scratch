#ifndef HOMOGRAPHY_H
#define HOMOGRAPHY_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <opencv2/core/types_c.h>

namespace homography
{
    void get_homography_matrix(std::vector<std::vector<cv::Point2f>> &object_points,
                                std::vector<std::vector<cv::Point2f>> &img_points, 
                                std::vector<Eigen::Matrix3d> &Hn);

    Eigen::Matrix3d estimate_frame_homography(std::vector<cv::Point2f> &object_points,
                                std::vector<cv::Point2f> &img_points);
}

#endif