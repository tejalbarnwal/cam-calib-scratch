#include "../include/homography.h"

#include <iostream>
#include <eigen3/Eigen/Dense>

namespace homography
{
    void get_homography_matrix(Eigen::MatrixXd *X, Eigen::MatrixXd *x, int num_imgs, Eigen::MatrixXd *Hn)
    {
        std::cout << "calculate homography matrix\n";
        // for each img homography matrix would contain 9 params
        // therefore, size of homography matrix can be: no of img * 9
        int num_corners_each_img = ((x->size())/2.0)/num_imgs;

        if (Hn->size() == 0)
        {
            Hn->resize(num_imgs, 9);
        }
        // for(int i=0; i< x->rows(); i++)
        // {
        //     std::cout << "vector coord: " << i/(object_points[0]).size() << " , "  << i%(object_points[0]).size() << "\n";
        //     (*X)(i, 0) = (object_points[i/(object_points[0]).size()][i%(object_points[0]).size()]).x;
        //     (*X)(i, 1) = (object_points[i/(object_points[0]).size()][i%(object_points[0]).size()]).y;
        // }
        for(int i=0; i<Hn->rows(); i++)
        {
            std::cout << "i: " << i << "\n";
            // i img X
            Eigen::MatrixXd X_i = X->block(i*42, 0, 42, X->cols());
            std::cout << "X_i: " << X_i << "\n\n";
            // i img x
            Eigen::MatrixXd x_i = x->block(i*42, 0, 42, x->cols());
            std::cout << "x_i: " << x_i << "\n";
            // estimate_frame_homography()
        }
    }

    void estimate_frame_homography(Eigen::MatrixXd *X, Eigen::MatrixXd *x, int num_imgs)
    {
        int num_corners_each_img = ((x->size())/2.0)/num_imgs;
        std::cout << "calculate homography matrix for each img\n";

        


    }
}