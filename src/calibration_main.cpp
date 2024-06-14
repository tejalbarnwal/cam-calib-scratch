#include <iostream>
#include <utility>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/types_c.h>

#include "../include/calib/compute_init_param.h"
#include "../include/calib/refine.h"


void findChessBoardCorner(std::string folder_path,
                                std::pair<int, int> pattern_size,
                                double square_size,
                                bool show_img,
                                std::vector<std::vector<cv::Point2f>> &object_points,
                                std::vector<std::vector<cv::Point2f>> &img_points)
{
    cv::Size chessBoardSize(pattern_size.first, pattern_size.second);

    std::cout << "\n-- Detecting Corners in each img --\n";
    std::vector<std::string> img_files;
    cv::glob(folder_path, img_files, false);

    size_t count = img_files.size();

    std::vector<cv::Point2f> corners;
    std::vector<cv::Point2f> obj;
    for (int i=0; i<(pattern_size.first * pattern_size.second); i++)
    {
        obj.push_back(cv::Point2f(
                                    (float)((i%pattern_size.first) * square_size), 
                                    (float)((i/pattern_size.first) * square_size)
                                    ));
    }
    // for(int i=0; i<obj.size(); i++)
    // {
    //     std::cout << obj[i] << "\n";
    // }
    // std::cout << "\n\n\n";

    for(size_t i=0; i < count; i++)
    {
        std::cout << "-- reading img " << img_files[i] << "--\n";
        cv::Mat img = cv::imread(img_files[i], cv::IMREAD_GRAYSCALE);
        if (img.empty())
        {
            std::cerr << "could not read image: " << img_files[i] << "\n";
            continue;
        }
        // std::cout << "img size: " << img.size().height << img.size().width << "\n";
        if (show_img)
        {
            cv::imshow("Image", img);
            int k = cv::waitKey(500);
        }

        bool corners_found = cv::findChessboardCorners(img, chessBoardSize, corners);

        if (corners_found)
        {
            // cv::cornerSubPix(img, corners, cv::Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            // std::cout << "corners found\n";
            // std::cout << "Number of corners: " << corners.size() << "\n";
            cv::cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));
            cv::drawChessboardCorners(img, chessBoardSize, corners, corners_found);
            
            if (show_img)
            {
                cv::imshow("Image", img);
                int k = cv::waitKey(500);
                cv::destroyAllWindows();
            }
            

            img_points.push_back(corners);
            object_points.push_back(obj);

            // std::cout << "corners: \n";
            // for(int i=0; i<corners.size(); i++)
            // {
            //     std::cout << corners[i] << "\n";
            // }
            // std::cout << "obj: \n";
            // for(int i=0; i<obj.size(); i++)
            // {
            //     std::cout << obj[i] << "\n";
            // }
        }
        else
        {
            std::cout << "corner not found in " << img_files[i] << "\n";
        }
    }
}


int main(int argc, char const *argv[])
{
    std::cout << "\n-- Calibration Source File --\n";

    std::string abs_path_img_folder = "/home/radiant/calib/references/cam-calib-scratch/chessboard_data/*.jpg";
    std::pair<int, int> pattern_size = std::make_pair(7, 6);
    double square_size = 1.0;
    bool show_corners_on_img = false;

    std::vector<std::vector<cv::Point2f>> img_points;
    std::vector<std::vector<cv::Point2f>> object_points;

    findChessBoardCorner(abs_path_img_folder,
                        pattern_size,
                        square_size,
                        show_corners_on_img,
                        object_points, img_points);

    std::vector<Eigen::Matrix3d> Hn;
    Eigen::Matrix3d K;
    std::vector<Eigen::Matrix3d> Rn;
    std::vector<Eigen::Vector3d> tn;

    computeInitialParams initialize_calib_params(object_points, img_points);
    if (initialize_calib_params.compute_homography())
    {
        initialize_calib_params.getHomography(Hn);

        if(initialize_calib_params.compute_intrinsics())
        {
            initialize_calib_params.getIntrinsics(K);

            if(initialize_calib_params.compute_extrinsics())
            {
                initialize_calib_params.getExtrinsics(Rn, tn);
            }
        }
    }

    Eigen::Vector2d d;
    d << 0.0, 0.0;
    // d << -0.298820, 0.138997 ;

    std::cout << "K init: " << K << "\n"; // intrinsic matrix
    std::cout << "d init: " << d.transpose() << "\n"; // radial distortion param
    // tn-> tranlation extrinsic param, Rn->rotation extrinsic param

    Eigen::VectorXd P_init = initialize_calib_params.composeP(K, Rn, tn, d);
    std::cout << "P init: " << P_init << "\n";
    int P_size = P_init.size();
    // std::cout << "P size: " << P_size << "\n";
    std::cout << "\n------------- setup and run lm optimization --------------\n";
    typedef refine::LMFunctor<double> functorEg;
    functorEg functor(P_size, 462);
    functor.img_points = img_points;
    functor.object_points = object_points;
    std::cout << "functor created\n";
    Eigen::NumericalDiff<functorEg> numDiff(functor);
    std::cout << "num diff created\n";
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<functorEg>, double> lm(numDiff);
    
    std::cout << "run lm\n";
    lm.parameters.maxfev = 100000;
    lm.parameters.ftol= 1.0e-8;
    lm.parameters.gtol = 1.0e-8;
    lm.parameters.xtol = 1.0e-8;

    Eigen::VectorXd P = P_init;
    int ret = lm.minimize(P);
    // std::cout << "fvec : " << lm.fvec << std::endl;
    std::cout << "iter count: " << lm.iter << std::endl;
    std::cout << "iter cost function: " << lm.fnorm << std::endl;
    std::cout << "return status: " << ret << std::endl;
    std::cout << "After solver, resultant P: \n" << P.transpose() << std::endl;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;


    Eigen::Matrix3d optK;
    std::vector<Eigen::Matrix3d> optRn;
    std::vector<Eigen::Vector3d> optTn;
    Eigen::Vector2d optD;
    initialize_calib_params.decomposeP(P, optK, optRn, optTn, optD);

    std::cout << "\noptK: " << optK << "\n";
    std::cout << "\noptD: " << optD.transpose() << "\n";

    return 0;
}