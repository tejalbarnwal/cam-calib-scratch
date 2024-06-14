#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>

#include <opencv2/core/types_c.h>


class computeInitialParams
{
private:
    // object points: corners on chess board plane in world coordinates
    // img points: corners seen as img indices
    std::vector<std::vector<cv::Point2f>> object_points, img_points;

    // homography matrix for each img
    std::vector<Eigen::Matrix3d> Hn;

    // intial intrinsic matrix
    Eigen::Matrix3d K;

    // initial extrinsic matrix containing Rn as rotation matrix and tn as translation vector
    std::vector<Eigen::Matrix3d> Rn;
    std::vector<Eigen::Vector3d> tn;

    // helper method to estimate normalization matrix
    Eigen::Matrix3d get_normalization_matrix(const std::vector<cv::Point2f> &vec_);

    // helper method to estimate homography for each img
    std::pair<bool, Eigen::Matrix3d> estimate_frame_homography(const std::vector<cv::Point2f> &object_points_,
                                                const std::vector<cv::Point2f> &img_points_);

    // helper method to compute Vpq vector for each img
    void make_Vpq(int p_, int q_, const Eigen::Matrix3d &H_, Eigen::VectorXd &Vi_);

    // helper method to compute extrinsics for each img
    void estimate_view_transform(const Eigen::Matrix3d &H_,  
                                Eigen::Matrix3d &R_, 
                                Eigen::Vector3d &t_);
    
    // helper method to re-compute R to ensure special orthogonal properties
    Eigen::Matrix3d make_true_rotation_matrix(const Eigen::Matrix3d &R_);

public:
    computeInitialParams(const std::vector<std::vector<cv::Point2f>> object_points_,
                        const std::vector<std::vector<cv::Point2f>> img_points_);

    ~computeInitialParams();

    bool compute_homography();

    bool compute_intrinsics();

    bool compute_extrinsics();

    void getHomography(std::vector<Eigen::Matrix3d> &Hn_);

    void getIntrinsics(Eigen::Matrix3d &K_);

    void getExtrinsics(std::vector<Eigen::Matrix3d> &Rn_, std::vector<Eigen::Vector3d> &tn_);

    Eigen::VectorXd composeP(const Eigen::Matrix3d &K,
                    const std::vector<Eigen::Matrix3d> &Rn,
                    const std::vector<Eigen::Vector3d> &tn,
                    const Eigen::Vector2d &d);

    void decomposeP(const Eigen::VectorXd &optP,
                    Eigen::Matrix3d &optK,
                    std::vector<Eigen::Matrix3d> &optRn,
                    std::vector<Eigen::Vector3d> &optTn,
                    Eigen::Vector2d &optD);
};