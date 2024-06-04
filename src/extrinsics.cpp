#include "../include/extrinsics.h"

namespace extrinsics
{
    Eigen::Matrix3d make_true_rotation_matrix(Eigen::Matrix3d &R)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd((R), Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::VectorXd singularValues = svd.singularValues();
        Eigen::MatrixXd V = svd.matrixV();
        Eigen::MatrixXd U = svd.matrixU();

        std::cout << "U: " << U << "\n";
        std::cout << "V: " << V.transpose() << "\n";

        Eigen::Matrix3d R_ = U * (V.transpose());

        std::cout << "R_: " << R_ << "\n";

        return R_;
    }
    
    void estimate_view_transform(Eigen::Matrix3d &H, 
                                Eigen::Matrix3d &K, 
                                Eigen::Matrix3d &R, 
                                Eigen::Vector3d &t)
    {
        Eigen::Vector3d h0 = H.col(0);
        Eigen::Vector3d h1 = H.col(1);
        Eigen::Vector3d h2 = H.col(2);

        std::cout << "h0: " << h0 << std::endl;
        std::cout << "h1: " << h1 << std::endl;
        std::cout << "h2: " << h2 << std::endl;

        double k = (1.0 / (K.inverse() * h0).norm());
        Eigen::Vector3d r0 = k * (K.inverse() * h0);
        Eigen::Vector3d r1 = k * (K.inverse() * h1);
        Eigen::Vector3d r2 = r0.cross(r1);
        t = k * (K.inverse() * h2);

        std::cout << "k: " << k << std::endl;
        std::cout << "r0: " << r0 << std::endl;
        std::cout << "r1: " << r1 << std::endl;
        std::cout << "r2: " << r2 << std::endl;
        std::cout << "t: " << t << std::endl;

        Eigen::Matrix3d R_init;
        R_init.col(0) = r0;
        R_init.col(1) = r1;
        R_init.col(2) = r2;

        std::cout << "R init: " << R_init << std::endl;

        R = make_true_rotation_matrix(R_init);
        std::cout << "R: " << R << "\n";
    }
    
    
    void get_camera_extrinsics(std::vector<Eigen::Matrix3d> &Hn, 
                                Eigen::Matrix3d &K,
                                std::vector<Eigen::Matrix3d> &Rn,
                                std::vector<Eigen::Vector3d> &tn)
    {
        std::cout << "\n\n\n-------------------\n\n\n";
        int num_imgs = Hn.size();

        for(int i=0; i < num_imgs; i++)
        {
            std::cout << "---------------------------\n";
            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            estimate_view_transform(Hn[i], K, R, t);
            Rn.push_back(R);
            tn.push_back(t);
        }
    }
}
