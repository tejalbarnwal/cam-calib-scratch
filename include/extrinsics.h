#ifndef EXTRINSICS_H
#define EXTRINSICS_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace extrinsics
{
    void get_camera_extrinsics(std::vector<Eigen::Matrix3d> &Hn, 
                                Eigen::Matrix3d &K,
                                std::vector<Eigen::Matrix3d> &Rn,
                                std::vector<Eigen::Vector3d> &tn);
    // void make_Vpq(int p, int q, Eigen::Matrix3d &H, Eigen::VectorXd &Vi);
}

#endif