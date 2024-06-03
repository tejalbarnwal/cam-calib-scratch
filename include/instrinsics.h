#ifndef INTRINSICS_H
#define INTRINSICS_H

#include <iostream>
#include <eigen3/Eigen/Dense>

namespace intrinsics
{
    void get_camera_intrinsics(Eigen::MatrixXd *Hn);
}

#endif