#include "../../include/calib/compute_init_param.h"

computeInitialParams::computeInitialParams(const std::vector<std::vector<cv::Point2f>> object_points_,
                                        const std::vector<std::vector<cv::Point2f>> img_points_)
{
    object_points = object_points_;
    img_points = img_points_;
}

computeInitialParams::~computeInitialParams()
{}

Eigen::Matrix3d 
computeInitialParams::get_normalization_matrix(const std::vector<cv::Point2f> &vec_)
{
    int size = vec_.size();
    Eigen::VectorXd vec_x(size);
    Eigen::VectorXd vec_y(size);

    for(int i=0; i<size; i++)
    {
        vec_x(i) = (vec_[i]).x;
        vec_y(i) = (vec_[i]).y;
    }

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

std::pair<bool, Eigen::Matrix3d> 
computeInitialParams::estimate_frame_homography(const std::vector<cv::Point2f> &object_point,
                                                const std::vector<cv::Point2f> &img_point)
{
    std::cout << "calculate homography matrix for each img\n";

    if (object_point.size() != img_point.size())
    {
        std::cerr << "No of point on the img do not match the no of 3D object points\n";
        return std::make_pair(false, Eigen::Matrix3d::Zero());
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

        return std::make_pair(true, H_mat);
    }
}

bool computeInitialParams::compute_homography()
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
        std::pair<bool, Eigen::Matrix3d> result = estimate_frame_homography(obj_vector, img_vector);
        Hn.push_back(result.second);
        
    }
    // std::cout << "Hn: " << (*Hn) << "\n";
    return true;
}

void
computeInitialParams::make_Vpq(int p, int q, const Eigen::Matrix3d &H, Eigen::VectorXd &Vi)
{
    Vi << H(0, p) * H(0, q),
        H(0, p) * H(1, q) + H(1, p) * H(0, q),
        H(1, p) * H(1, q),
        H(2, p) * H(0, q) + H(0, p) * H(2, q),
        H(2, p) * H(1, q) + H(1, p) * H(2, q),
        H(2, p) * H(2, q) ;
}

bool computeInitialParams::compute_intrinsics()
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

    K = K_inv_T(2, 2) * (K_inv_T.inverse()).transpose();

    std::cout << "K: " << K << "\n";
    return true;
}

Eigen::Matrix3d computeInitialParams::make_true_rotation_matrix(const Eigen::Matrix3d &R)
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

void computeInitialParams::estimate_view_transform(const Eigen::Matrix3d &H,  
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

bool computeInitialParams::compute_extrinsics()
{
    std::cout << "\n\n\n-------------------\n\n\n";
    int num_imgs = Hn.size();

    for(int i=0; i < num_imgs; i++)
    {
        std::cout << "---------------------------\n";
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        estimate_view_transform(Hn[i], R, t);
        Rn.push_back(R);
        tn.push_back(t);
    }
    return true;
}

void 
computeInitialParams::getHomography(std::vector<Eigen::Matrix3d> &Hn_)
{
    Hn_ = Hn;
}

void 
computeInitialParams::getIntrinsics(Eigen::Matrix3d &K_)
{
    K_ = K;
}

void 
computeInitialParams::getExtrinsics(std::vector<Eigen::Matrix3d> &Rn_, std::vector<Eigen::Vector3d> &tn_)
{
    Rn_ = Rn;
    tn_ = tn;
}

Eigen::VectorXd 
computeInitialParams::composeP(const Eigen::Matrix3d &K,
                                const std::vector<Eigen::Matrix3d> &Rn,
                                const std::vector<Eigen::Vector3d> &tn,
                                const Eigen::Vector2d &d)
{
    std::cout << "\n\n----------------opt begin ---------------\n\n";
    int num_imgs = Rn.size();
    Eigen::VectorXd P(7 + 6 * num_imgs);
    Eigen::VectorXd k(7); 
    k << K(0, 0) , K(1, 1), K(0, 1), K(0, 2), K(1, 2), d[0], d[1];
    P.head(7) = k;

    for(int i = 0; i < num_imgs; i++)
    {
        Eigen::AngleAxisd angleAxis(Rn[i]);
        Eigen::Vector3d rodriguesVector = angleAxis.angle() * angleAxis.axis();

        std::cout << "R: " << Rn[i] << "\n";
        std::cout << "rodrigues vector: " << rodriguesVector.transpose() << "\n";

        Eigen::VectorXd w(6);
        w << rodriguesVector(0), rodriguesVector(1), rodriguesVector(2), (tn[i])(0), (tn[i])(1), (tn[i])(2);
        std::cout << "w: " << w << "\n";
        P.segment(7 + 6 * i, 6) = w;
    }

    return P;
}


void 
computeInitialParams::decomposeP(const Eigen::VectorXd &optP,
                                Eigen::Matrix3d &optK,
                                std::vector<Eigen::Matrix3d> &optRn,
                                std::vector<Eigen::Vector3d> &optTn,
                                Eigen::Vector2d &optD)
{
    double alpha = optP(0);
    double beta = optP(1);
    double gamma = optP(2);
    double uc = optP(3);
    double vc = optP(4);
    double k0 = optP(5);
    double k1 = optP(6);

    optK << alpha, gamma,  uc, 
        0.0,    beta,   vc, 
        0.0,    0.0,    1.0;
    
    optD(0) = k0;
    optD(1) = k1;

    int num_imgs = (optP.size() - 7) / 6;
    std::cout << "num imgs: " << num_imgs;

    for(int i=0; i< num_imgs; i++)
    {
        int m = 7 + 6 * i;
        Eigen::Vector3d optRodriguesVector = optP.segment(m, 3);
        double angle = optRodriguesVector.norm();
        Eigen::Vector3d axis = optRodriguesVector.normalized();
        Eigen::AngleAxisd angleAxis(angle, axis);

        Eigen::Vector3d t = optP.segment(m+3, 3);
        Eigen::Matrix3d R = angleAxis.toRotationMatrix();

        optRn.push_back(R);
        optTn.push_back(t);
    }
}