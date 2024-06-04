#include "../include/instrinsics.h"

#include <iostream>
#include <eigen3/Eigen/Dense>

namespace intrinsics
{
    double getH(int p, int q, Eigen::Matrix3d *Hi)
    {
        return (*Hi)(3*p+q);
    }


    void make_Vpq(int p, int q, Eigen::Matrix3d &H, Eigen::VectorXd &Vi)
    {
        Vi << H(0, p) * H(0, q),
                H(0, p) * H(1, q) + H(1, p) * H(0, q),
                H(1, p) * H(1, q),
                H(2, p) * H(0, q) + H(0, p) * H(2, q),
                H(2, p) * H(1, q) + H(1, p) * H(2, q),
                H(2, p) * H(2, q) ;
    }
    
    void get_camera_intrinsics(std::vector<Eigen::Matrix3d> &Hn)
    {
        int num_imgs = Hn.size();
        std::cout << "\nnum_imgs: " << num_imgs << "\n";
        // ensure we have enough points to solve this system
        Eigen::MatrixXd Vn(2 * num_imgs, 6);

        for(int i=0; i<num_imgs; i++)
        {
            Eigen::Matrix3d H_i = Hn[i];
            Eigen::VectorXd V0(6);
            make_Vpq(0, 1, H_i, V0);
            Vn.row(2*i) = V0;

            Eigen::VectorXd V1(6);
            Eigen::VectorXd V2(6);
            make_Vpq(0, 0, H_i, V1);
            make_Vpq(1, 1, H_i, V2);
            Vn.row(2*i+1) = V1 - V2;
        }
        Eigen::VectorXd b;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd((Vn), Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::VectorXd singularValues = svd.singularValues();
        Eigen::MatrixXd V = svd.matrixV();
        Eigen::MatrixXd U = svd.matrixU();

        // int index = 0;
        // double minSingularValue = singularValues.minCoeff(&index);
        // Eigen::VectorXd leastSingularVector = V.row(index);
        Eigen::VectorXd leastSingularVector = V.col(V.cols() - 1);

        // std::cout << "Original matrix M:\n" << (Vn) << std::endl << std::endl;
        // std::cout << "Singular values:\n" << singularValues << std::endl << std::endl;
        // std::cout << "Right singular vectors (V):\n" << V << std::endl << std::endl;
        // std::cout << "Vector with least singular value:\n" << leastSingularVector << std::endl;

        std::cout << "\n\n\n--------";
        b = leastSingularVector;
        std::cout << "Vector with least singular value:\n" << b << std::endl;

        Eigen::Matrix3d B;
        B(0, 0) = b(0);
        B(0, 1) = b(1);
        B(1, 0) = b(1);
        B(0, 2) = b(3);
        B(2, 0) = b(3);
        B(1, 1) = b(2);
        B(1, 2) = b(4);
        B(2, 1) = b(4);
        B(2, 2) = b(5);

        std::cout << "B: " << B << std::endl;

        Eigen::Matrix3d K_inv_T;

        Eigen::LLT<Eigen::Matrix3d> choleskyDecomposeofB(B);
        if (choleskyDecomposeofB.info() == Eigen::Success)
        {
            K_inv_T = choleskyDecomposeofB.matrixL();
        }
        else
        {
            // make it better
            std::cout << "\n-- doing cholesky decomposition for -B\n";
            Eigen::LLT<Eigen::Matrix3d> choleskyDecomposeofB(-1*B);
            K_inv_T = choleskyDecomposeofB.matrixL();
        }

        std::cout << "K_inv_T: " << K_inv_T << "\n";

        Eigen::Matrix3d K = K_inv_T(2, 2) * (K_inv_T.inverse()).transpose();

        std::cout << "K: " << K << "\n";


        
    }

}