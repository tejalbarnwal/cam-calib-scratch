#ifndef INTRINSICS_H
#define INTRINSICS_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace intrinsics
{
    Eigen::Matrix3d get_camera_intrinsics(std::vector<Eigen::Matrix3d> &Hn);
    void make_Vpq(int p, int q, Eigen::Matrix3d &H, Eigen::VectorXd &Vi);
}

#endif