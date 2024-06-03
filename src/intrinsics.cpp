#include "../include/instrinsics.h"

#include <iostream>
#include <eigen3/Eigen/Dense>

namespace intrinsics
{
    double getH(int p, int q, Eigen::VectorXd *Hi)
    {
        return (*Hi)(3*p+q);
    }


    void make_Vpq(int p, int q, Eigen::VectorXd *Hi, Eigen::VectorXd *Vi)
    {
        (*Vi) << getH(0, p, Hi) * getH(0, q, Hi), 
                    getH(0, p, Hi) * getH(1, q, Hi) + getH(1, p, Hi) * getH(0, q, Hi), 
                    getH(1, p, Hi) * getH(1, q, Hi),
                    getH(2, p, Hi) * getH(0, q, Hi) + getH(0, p, Hi) * getH(2, q, Hi),
                    getH(2, p, Hi) * getH(1, q, Hi) + getH(1, p, Hi) * getH(2, q, Hi),
                    getH(2, p, Hi) * getH(2, q, Hi) ;
    }
    
    void get_camera_intrinsics(Eigen::MatrixXd *Hn)
    {
        int num_imgs = Hn->rows();
        std::cout << "\nnum_imgs: " << num_imgs << "\n";
        // ensure we have enough point to solve this system
        Eigen::MatrixXd Vn;

        if (Vn.size() == 0)
        {
            Vn.resize(2 * num_imgs, 6);
        }
        for(int i=0; i<num_imgs; i++)
        {
            std::cout << "i: " << i << "\n";

            Eigen::VectorXd H_i = Hn->row(i);
            Eigen::VectorXd V_i = Vn.row(2*i);
            make_Vpq(0, 1, &H_i, &V_i);
            Vn.row(2*i) = V_i;

            Eigen::VectorXd V1(6);
            Eigen::VectorXd V2(6);
            make_Vpq(0, 0, &H_i, &V1);
            make_Vpq(1, 1, &H_i, &V2);
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
        B(1, 1) = b(2);
        B(2, 0) = b(2);
        B(1, 2) = b(4);
        B(2, 1) = b(4);
        B(2, 2) = b(5);

        Eigen::Matrix3d K;

        Eigen::LLT<Eigen::Matrix3d> choleskyDecomposeofB(B);
        if (choleskyDecomposeofB.info() == Eigen::Success)
        {
            K = choleskyDecomposeofB.matrixL();
        }
        else
        {
            std::cout << "\n-- doing cholesky decomposition for -B\n";
            // Eigen::LLT<Eigen::Matrix3d> choleskyDecomposeofB(-1*B);
            // K = choleskyDecomposeofB.matrixL();
        }
        
    }

}