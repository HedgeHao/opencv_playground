#include <iostream>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define PREVIEW_WINDOW "Preview"
#define CALIBRATE_WINDOW "Calibration"
#define CALIBRATE_NUM 10

int main(int argc, char **argv)
{
    std::cout << cv::getVersionString() << std::endl;

    std::vector<cv::String> fileNames;
    cv::glob("../imgs/*.jpg", fileNames, false);
    cv::Size patternSize(15, 9);
    std::vector<std::vector<cv::Point2f>> q(fileNames.size());

    std::vector<std::vector<cv::Point3f>> Q;
    // 1. Generate checkerboard (world) coordinates Q. The board has 25 x 18
    // fields with a size of 15x15mm

    int checkerBoard[2] = {16, 10};
    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < checkerBoard[1] - 1; i++)
    {
        for (int j = 0; j < checkerBoard[0] - 1; j++)
        {
            objp.push_back(cv::Point3f(j, i, 0));
        }
    }

    std::vector<cv::Point2f> imgPoint;
    // Detect feature points
    cv::Mat img;
    std::size_t i = 0;
    for (auto const &f : fileNames)
    {
        std::cout << std::string(f) << std::endl;

        // 2. Read in the image an call cv::findChessboardCorners()
        img = cv::imread(fileNames[i]);
        cv::Mat gray;

        cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

        bool patternFound = cv::findChessboardCorners(gray, patternSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        // 2. Use cv::cornerSubPix() to refine the found corner detections
        if (patternFound)
        {
            cv::cornerSubPix(gray, q[i], cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            Q.push_back(objp);
        }

        // Display
        cv::drawChessboardCorners(img, patternSize, q[i], patternFound);
        cv::imshow("chessboard detection", img);
        cv::waitKey(100);

        i++;
    }

    cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
                cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
    std::cout << img.cols << "," << img.rows << std::endl;
    cv::Size frameSize(1920, 1080);

    std::cout << "Calibrating..." << std::endl;
    // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
    // and output parameters as declared above...

    double error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);

    std::vector<cv::Point2f> projectPoints;
    double mean_error = 0.0;
    for (int i = 0; i < Q.size(); i++)
    {
        cv::projectPoints(Q[i], rvecs[i], tvecs[i], K, k, projectPoints);
        mean_error += cv::norm(q[i], projectPoints, cv::NORM_L2) / projectPoints.size();
    }

    std::cout << "mean error = " << mean_error / objp.size() << "\n"
              << "Reprojection error = " << error << "\nK =\n"
              << K << "\nk=\n"
              << k << std::endl;

    return 0;
}