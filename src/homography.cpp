#include "../include/homography.h"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>

namespace homography
{
    void get_homography_matrix(std::vector<std::vector<cv::Point2f>> &object_points,
                                std::vector<std::vector<cv::Point2f>> &img_points, 
                                std::vector<Eigen::Matrix3d> &Hn)
    {
        std::cout << "calculate homography matrix\n";
        // for each img homography matrix would contain 9 params
        // therefore, size of homography matrix can be: no of img * 9
        int num_imgs = object_points.size();
        int num_corners_each_img = object_points[0].size();

        // for(int i=0; i< x->rows(); i++)
        // {
        //     std::cout << "vector coord: " << i/(object_points[0]).size() << " , "  << i%(object_points[0]).size() << "\n";
        //     (*X)(i, 0) = (object_points[i/(object_points[0]).size()][i%(object_points[0]).size()]).x;
        //     (*X)(i, 1) = (object_points[i/(object_points[0]).size()][i%(object_points[0]).size()]).y;
        // }
        for(int i=0; i < num_imgs; i++)
        {
            // std::cout << "i: " << i << "\n";
            // i img X
            // Eigen::MatrixXd X_i = X->block(i*42, 0, 42, X->cols());
            // std::cout << "X_i: " << X_i << "\n\n";
            // i img x
            // Eigen::MatrixXd x_i = x->block(i*42, 0, 42, x->cols());
            // std::cout << "x_i: " << x_i << "\n";

            std::vector<cv::Point2f> &img_vector = img_points[i];
            std::vector<cv::Point2f> &obj_vector = object_points[i];

            // Eigen::VectorXd H_i = Hn->row(i);
            Hn.push_back(estimate_frame_homography(obj_vector, img_vector));
            
        }
        // std::cout << "Hn: " << (*Hn) << "\n";
    }

    Eigen::Matrix3d get_normalization_matrix(std::vector<cv::Point2f> &vec)
    {
        int size = vec.size();
        Eigen::VectorXd vec_x(size);
        Eigen::VectorXd vec_y(size);

        for(int i=0; i<size; i++)
        {
            vec_x(i) = (vec[i]).x;
            vec_y(i) = (vec[i]).y;
        }

        // std::cout << "vec x: " << vec_x << "\n";
        // std::cout << "vec y: " << vec_y << "\n";

        double mean_x = vec_x.mean();
        std::cout << "mean_x: " << mean_x << "\n";
        double mean_y = vec_y.mean();
        std::cout << "mean_y: " << mean_y << "\n";

        double var_x = (vec_x.array() - mean_x).square().mean();
        std::cout << "var_x: " << var_x << "\n";

        double var_y = (vec_y.array() - mean_y).square().mean();
        std::cout << "var_y: " << var_y << "\n";

        double sd_x = std::sqrt(2.0 / var_x);
        std::cout << "s_x: " << sd_x << "\n";

        double sd_y = std::sqrt(2.0 / var_y);
        std::cout << "s_y: " << sd_y << "\n";


        Eigen::Matrix3d norm;

        norm << sd_x,     0.0,    -1.0 * sd_x * mean_x,
                0.0,        sd_y,   -1.0 * sd_y * mean_y,
                0.0,        0.0,    1.0;
        
        return norm;

    }

    Eigen::Matrix3d estimate_frame_homography(std::vector<cv::Point2f> &object_point,
                                std::vector<cv::Point2f> &img_point)
    {
        // int num_corners_each_img = ((x->size())/2.0)/num_imgs;
        std::cout << "calculate homography matrix for each img\n";

        if (object_point.size() != img_point.size())
        {
            std::cerr << "No of point on the img do not match the no of 3D object points\n";
            return Eigen::Matrix3d::Zero();
        }
        else
        {
            int num_corners = object_point.size() ;

            Eigen::Matrix3d object_norm = get_normalization_matrix(object_point);
            std::cout << "object norm: " << object_norm << "\n";
            Eigen::Matrix3d img_norm = get_normalization_matrix(img_point);
            std::cout << "img norm: " << img_norm << "\n";

            // Eigen::MatrixXd *M = new Eigen::MatrixXf(Eigen::MatrixXf::Zero(2*num_corners, 9));
            Eigen::MatrixXd M = Eigen::MatrixXd::Zero(2*num_corners, 9);
            for (int i=0; i<num_corners; i++)
            {
                int k = 2*i;
                double X0 = ((object_point)[i]).x;
                std::cout << "X0:" << X0 << "\n";
                double Y0 = ((object_point)[i]).y;
                std::cout << "Y0:" << Y0 << "\n";
                double x0 = ((img_point)[i]).x;
                std::cout << "x0:" << x0 << "\n";
                double y0 = ((img_point)[i]).y;
                std::cout << "y0:" << y0 << "\n";

                Eigen::Vector3d X{((object_point)[i]).x, ((object_point)[i]).y, 1.0};
                Eigen::Vector3d x{((img_point)[i]).x, ((img_point)[i]).y, 1.0};

                Eigen::Vector3d norm_X = object_norm * X;
                Eigen::Vector3d norm_x = img_norm * x;
                Eigen::Vector2d norm_X_2d{norm_X(0)/norm_X(2), norm_X(1)/norm_X(2)};
                Eigen::Vector2d norm_x_2d{norm_x(0)/norm_x(2), norm_x(1)/norm_x(2)};

                std::cout << "norm_X_2d: " << norm_X_2d << "\n";
                std::cout << "norm_x_2d: " << norm_x_2d << "\n";

                // M.row(k) << -X0, -Y0, -1, 0.0, 0.0, 0.0, x0*X0, x0* X0, x0;
                // M.row(k+1) << 0.0, 0.0, 0.0, -X0, -Y0, -1, y0*X0, y0*Y0, y0;
                M.row(k) << -norm_X_2d(0), -norm_X_2d(1), -1.0, 0.0, 0.0, 0.0, norm_x_2d(0) * norm_X_2d(0), norm_x_2d(0) * norm_X_2d(1), norm_x_2d(0);
                M.row(k+1) << 0.0, 0.0, 0.0, -norm_X_2d(0), -norm_X_2d(1), -1.0, norm_x_2d(1) * norm_X_2d(0), norm_x_2d(1) * norm_X_2d(1), norm_x_2d(1);
            }

            Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::VectorXd singularValues = svd.singularValues();
            Eigen::MatrixXd V = svd.matrixV();
            Eigen::MatrixXd U = svd.matrixU();

            // int index = 0;
            // double minSingularValue = singularValues.minCoeff(&index);
            // Eigen::VectorXd leastSingularVector = V.row(index);
            Eigen::VectorXd leastSingularVector = V.col(V.cols() - 1);

            // std::cout << "Original matrix M:\n" << M << std::endl << std::endl;
            // std::cout << "Singular values:\n" << singularValues << std::endl << std::endl;
            // std::cout << "Right singular vectors (V):\n" << V << std::endl << std::endl;
            std::cout << "Vector with least singular value:\n" << leastSingularVector << std::endl;

            std::cout << "\n\n\n--------";
            // (*H) = leastSingularVector;
            // std::cout << "Vector with least singular value:\n" << (*H) << std::endl;

            Eigen::Matrix3d sv_mat = (Eigen::Map<Eigen::Matrix3d>(leastSingularVector.data())).transpose();
            std::cout << "sv_mat: " << sv_mat << "\n";

            std::cout << "Q-1: " << img_norm.inverse() << "\n";

            std::cout << "Q-1 * nH: " << ((img_norm.inverse()) * sv_mat) << "\n";

            Eigen::Matrix3d H_mat = ((img_norm.inverse()) * sv_mat) * object_norm;

            std::cout << "h_mat: " << H_mat << "\n";

            // (*H) = Eigen::Map<Eigen::VectorXd>((H_mat.transpose()).data(), H_mat.size());

            // std::cout << "(H: )" << (*H) << "\n";

            return H_mat;


        }
    }
}