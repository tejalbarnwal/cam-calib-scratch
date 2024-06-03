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
            // std::cout << "X_i: " << X_i << "\n\n";
            // i img x
            Eigen::MatrixXd x_i = x->block(i*42, 0, 42, x->cols());
            // std::cout << "x_i: " << x_i << "\n";

            Eigen::VectorXd H_i = Hn->row(i);
            estimate_frame_homography(&X_i, &x_i, &H_i);
            Hn->row(i) = H_i;
        }
        std::cout << "Hn: " << (*Hn) << "\n";
    }

    void estimate_frame_homography(Eigen::MatrixXd *X, Eigen::MatrixXd *x, Eigen::VectorXd *H)
    {
        // int num_corners_each_img = ((x->size())/2.0)/num_imgs;
        std::cout << "calculate homography matrix for each img\n";

        if (X->size() != x->size())
        {
            std::cerr << "No of point on the img do not match the no of 3D object points\n";
        }
        else
        {
            int num_corners = X->size() / 2.0;

            // Eigen::MatrixXd *M = new Eigen::MatrixXf(Eigen::MatrixXf::Zero(2*num_corners, 9));
            Eigen::MatrixXd M = Eigen::MatrixXd::Zero(2*num_corners, 9);
            for (int i=0; i<num_corners; i++)
            {
                int k = 2*i;
                double X0 = (*X)(i, 0);
                double Y0 = (*X)(i, 1);
                double x0 = (*x)(i, 0);
                double y0 = (*x)(i, 1);
                M.row(k) << -X0, -Y0, -1, 0.0, 0.0, 0.0, x0*X0, x0* X0, x0;
                M.row(k+1) << 0.0, 0.0, 0.0, -X0, -Y0, -1, y0*X0, y0*Y0, y0;
            }

            Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::VectorXd singularValues = svd.singularValues();
            Eigen::MatrixXd V = svd.matrixV();
            Eigen::MatrixXd U = svd.matrixU();

            // int index = 0;
            // double minSingularValue = singularValues.minCoeff(&index);
            // Eigen::VectorXd leastSingularVector = V.row(index);
            Eigen::VectorXd leastSingularVector = V.col(V.cols() - 1);

            std::cout << "Original matrix M:\n" << M << std::endl << std::endl;
            std::cout << "Singular values:\n" << singularValues << std::endl << std::endl;
            std::cout << "Right singular vectors (V):\n" << V << std::endl << std::endl;
            std::cout << "Vector with least singular value:\n" << leastSingularVector << std::endl;

            std::cout << "\n\n\n--------";
            (*H) = leastSingularVector;
            std::cout << "Vector with least singular value:\n" << (*H) << std::endl;

        }
    }
}