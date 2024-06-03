#include "../include/calibration.h"
#include "../include/homography.h"
#include "../include/instrinsics.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

namespace calibration
{
    void findChessBoardCorner(std::string folder_path,
                                std::pair<int, int> pattern_size,
                                double square_size,
                                bool show_corners_on_img,
                                Eigen::MatrixXd *X, Eigen::MatrixXd *x)
    {
        // Eigen::MatrixXd frame_corner_indices(2, pattern_size.first * pattern_size.second);

        // // fill the grid matrix with row and coulmn indices
        // for(int c=0; c<pattern_size.second; c++){
        //     for(int r=0; r<pattern_size.first; r++)
        //     {
        //         frame_corner_indices(0, (c*pattern_size.first)+r) = c;
        //         frame_corner_indices(1, (c*pattern_size.first)+r) = r;
        //         // std::cout << "frame_corner_indices(0, " << (c*pattern_size.first)+r << ") : " << frame_corner_indices(0, (c*pattern_size.first)+r) << "\n";
        //     }
        // }

        // std::cout << frame_corner_indices << "\n";
        // frame_corner_indices = frame_corner_indices * square_size;

        
        cv::Size chessBoardSize(pattern_size.first, pattern_size.second);

        std::cout << "\n-- Detecting Corners in each img --\n";
        std::vector<std::string> img_files;
        cv::glob("/home/radiant/calib/references/cam-calib-scratch/chessboard_data/*.jpg", img_files, false);

        size_t count = img_files.size();

        std::vector<std::vector<cv::Point3f>> object_points;
        std::vector<std::vector<cv::Point2f>> img_points;
        std::vector<cv::Point2f> corners;
        std::vector<cv::Point3f> obj;
        for (int i=0; i<(pattern_size.first * pattern_size.second); i++)
        {
            obj.push_back(cv::Point3f(i/pattern_size.first, i%pattern_size.first, 0.0));
        }
        for(int i=0; i<obj.size(); i++)
        {
            std::cout << obj[i] << "\t";
        }
        std::cout << "\n\n\n";

        for(size_t i=0; i < count; i++)
        {
            std::cout << "\n\n-- reading img " << img_files[i] << "--\n";
            cv::Mat img = cv::imread(img_files[i], cv::IMREAD_GRAYSCALE);
            if (img.empty())
            {
                std::cerr << "could not read image: " << img_files[i] << "\n";
                continue;
            }
            // cv::imshow("Image", img);
            // int k = cv::waitKey(500);

            bool corners_found = cv::findChessboardCorners(img, chessBoardSize, corners);

            if (corners_found)
            {
                // cv::cornerSubPix(img, corners, cv::Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
                std::cout << "corners found\n";
                std::cout << "Number of corners: " << corners.size() << "\n";
                cv::cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));
                cv::drawChessboardCorners(img, chessBoardSize, corners, corners_found);
                // cv::imshow("Image", img);
                // int k = cv::waitKey(500);
                // cv::destroyAllWindows();

                img_points.push_back(corners);
                object_points.push_back(obj);

                for(int i=0; i<corners.size(); i++)
                {
                    std::cout << corners[i] << "\t";
                }
            }
            else
            {
                std::cout << "corner not found\n";
            }
        }

        std::cout << "cnvrt vector to eigen\n";

        // converts img_points vector and object_point vectors into eigen matrices
        if (img_points.empty())
        {
            std::cerr << "No corner points can be detected from given imags\n";
        }
        else
        {
            // size of eigen matrix
            // cols = 2, rows: no of img where corner detected * no of corners
            std::cout << "num img: " << img_points.size() << "\n";
            int rows = img_points.size() * (img_points[0]).size();
            std::cout << "rows in eigen matrices: " << rows << "\n";

            std::cout << X->size() << x->size() << "\n";
            if (x->size() == 0)
            {
                x->resize(rows, 2);
            }
            for(int i=0; i< x->rows(); i++)
            {
                std::cout << "vector coord: " << i/(img_points[0]).size() << " , "  << i%(img_points[0]).size() << "\n";
                (*x)(i, 0) = (img_points[i/(img_points[0]).size()][i%(img_points[0]).size()]).x;
                (*x)(i, 1) = (img_points[i/(img_points[0]).size()][i%(img_points[0]).size()]).y;
            }

            std::cout << "x: " << (*x) << "\n";

            std::cout << X->size() << x->size() << "\n";
            if (X->size() == 0)
            {
                X->resize(rows, 2);
            }
            for(int i=0; i< x->rows(); i++)
            {
                std::cout << "vector coord: " << i/(object_points[0]).size() << " , "  << i%(object_points[0]).size() << "\n";
                (*X)(i, 0) = (object_points[i/(object_points[0]).size()][i%(object_points[0]).size()]).x;
                (*X)(i, 1) = (object_points[i/(object_points[0]).size()][i%(object_points[0]).size()]).y;
            }

            std::cout << "X: " << (*X) << "\n";


        }
    }

    void calibrate(Eigen::MatrixXd *X, Eigen::MatrixXd *x, Eigen::MatrixXd *H_n)
    {
        std::cout << "\n-- Begin process of calibration --\n";
        int n = 11;
        homography::get_homography_matrix(X, x, n, H_n);
        intrinsics::get_camera_intrinsics(H_n);
    }

}