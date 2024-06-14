#ifndef REFINE_H
#define REFINE_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <opencv2/core/types_c.h>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/unsupported/Eigen/NumericalDiff>


namespace refine
{
    Eigen::Vector2d project(const Eigen::Matrix3d &K, 
                            const Eigen::Matrix3d &R, 
                            const Eigen::Vector3d &t,
                            const Eigen::Vector2d &D, 
                            const Eigen::Vector4d &X)
    {
        Eigen::Matrix<double, 3, 4> W;
        W.block<3, 3>(0, 0) = R;
        W.block<3, 1>(0, 3) = t;
        Eigen::Vector3d x0 = W * X;

        Eigen::Vector2d x1(x0(0)/x0(2), x0(1)/x0(2));

        double r = x1.norm();
        double d = x1(0) * std::pow(r, 2) + x1(1) * std::pow(r, 4);
        Eigen::Vector2d x2 = x1 * (1 + d);

        Eigen::Vector3d x3(x2(0), x2(1), 1.0);
        Eigen::Vector3d x4 = K * x3;

        Eigen::Vector2d x5(x4(0)/x4(2), x4(1)/x4(2));

        return x5;
    }
    
    template<typename _Scalar>
    struct LMFunctor
    {
        std::vector<std::vector<cv::Point2f>> img_points;
        std::vector<std::vector<cv::Point2f>> object_points;

        typedef _Scalar Scalar;
        enum{
            InputsAtCompileTime = Eigen::Dynamic,
            ValuesAtCompileTime = Eigen::Dynamic
        };

        typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
        typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
        typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

        int no_of_inputs, no_of_values;

        LMFunctor() : no_of_inputs(InputsAtCompileTime), no_of_values(ValuesAtCompileTime) {}
        LMFunctor(int n_inputs, int n_values) : no_of_inputs(n_inputs), no_of_values(n_values) {}

        int inputs() const {return no_of_inputs; }
        int values() const {return no_of_values; }

        int operator()(const Eigen::VectorXd &P, Eigen::VectorXd &fvec) const
        {
            // std::cout << "operator called\n";
            double alpha = P(0);
            double beta = P(1);
            double gamma = P(2);
            double uc = P(3);
            double vc = P(4);
            double k0 = P(5);
            double k1 = P(6);

            Eigen::Matrix3d K;
            K << alpha, gamma,  uc, 
                    0.0,    beta,   vc, 
                    0.0,    0.0,    1.0;
            // std::cout << "K done\n";

            Eigen::Vector2d D;
            D(0) = k0;
            D(1) = k1;
            // std::cout << "D done\n";

            int num_imgs = (P.size() - 7) / 6;
            // std::cout << "num img: " << num_imgs << "\n";
            int num_points_each_img = object_points[0].size();
            // std::cout << "num_points_each_img: " << num_points_each_img << "\n";
            int l = 0;
            for(int i=0; i< num_imgs; i++)
            {
                int m = 7 + 6 * i;
                Eigen::Vector3d optRodriguesVector = P.segment(m, 3);
                double angle = optRodriguesVector.norm();
                Eigen::Vector3d axis = optRodriguesVector.normalized();
                Eigen::AngleAxisd angleAxis(angle, axis);

                Eigen::Vector3d t = P.segment(m+3, 3);
                Eigen::Matrix3d R = angleAxis.toRotationMatrix();

                std::vector<cv::Point2f> frame_object_points = object_points[i];
                std::vector<cv::Point2f> frame_img_points = img_points[i];

                for(int j=0; j < num_points_each_img; j++)
                {
                    Eigen::Vector4d X(frame_object_points[j].x, frame_object_points[j].y, 0.0, 1.0);
                    Eigen::Vector2d x = project(K, R, t, D, X);

                    fvec(i * num_imgs + j) =  pow(x(0) - frame_img_points[j].x, 2) + pow(x(1) - frame_img_points[j].y, 2);
                    l = l + 1;
                    // std::cout << "---------- l: " << l << "\n";
                }
            }
            // std::cout << fvec.size() << "\n";
            return 0;
        }
    };
}

#endif