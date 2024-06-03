#ifndef HOMOGRAPHY_H
#define HOMOGRAPHY_H

#include <iostream>
#include <eigen3/Eigen/Dense>

namespace homography
{
    void get_homography_matrix(Eigen::MatrixXd *X, 
                                Eigen::MatrixXd *x, 
                                int num_imgs, 
                                Eigen::MatrixXd *Hn);

    void estimate_frame_homography(Eigen::MatrixXd *X, 
                                Eigen::MatrixXd *x,
                                Eigen::VectorXd *H);
}

#endif