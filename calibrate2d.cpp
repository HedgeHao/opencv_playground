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
    int calibrate_num = 0;
    std::vector<std::vector<cv::Point3f>> Q;
    std::vector<std::vector<cv::Point2f>> q(CALIBRATE_NUM);

    cv::Size patternSize(16 - 1, 10 - 1);
    int checkerBoard[2] = {16, 10};
    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for (int i = 1; i < checkerBoard[1]; i++)
    {
        for (int j = 1; j < checkerBoard[0]; j++)
        {
            objp.push_back(cv::Point3f(j, i, 0));
        }
    }

    cv::namedWindow(PREVIEW_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(CALIBRATE_WINDOW, cv::WINDOW_AUTOSIZE);

    cv::VideoCapture cap(std::stoi(argv[1]));
    cv::Mat frame;
    char c;
    while (true)
    {
        bool ret = cap.read(frame);
        if (!ret)
            continue;

        cv::imshow(PREVIEW_WINDOW, frame);
        c = cv::waitKey(1);

        if (c == 'c')
        {
            if (calibrate_num >= CALIBRATE_NUM)
            {
                printf("Calibrate number is 10\n");
                continue;
            }

            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
            bool patternFound = cv::findChessboardCorners(gray, patternSize, q[calibrate_num], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

            if (patternFound)
            {
                cv::cornerSubPix(gray, q[calibrate_num], cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
                Q.push_back(objp);
                cv::drawChessboardCorners(frame, patternSize, q[calibrate_num], patternFound);
                cv::imshow(CALIBRATE_WINDOW, frame);

                calibrate_num++;
            }
        }
        else if (c == 27)
        {
            break;
        }
    }

    cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients
    cv::Size frameSize(frame.cols, frame.rows);
    std::cout << frame.cols << "," << frame.rows << std::endl;

    if (calibrate_num == 10)
    {
        std::vector<cv::Mat> rvecs, tvecs;
        std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
        int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
                    cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;

        printf("Calibrating...\n");

        float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);

        std::cout << "Reprojection error = " << error << "\nK =\n"
                  << K << "\nk=\n"
                  << k << std::endl;
    }

    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);
    while (true)
    {
        bool ret = cap.read(frame);
        if (!ret)
            continue;

        cv::Mat imgUndistorted;
        cv::remap(frame, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);

        cv::resize(frame, frame, cv::Size(640, 360));
        cv::resize(imgUndistorted, imgUndistorted, cv::Size(640, 360));

        std::vector<cv::Mat> imgs;
        cv::Mat result;
        imgs.push_back(frame);
        imgs.push_back(imgUndistorted);
        cv::hconcat(imgs, result);

        cv::imshow(PREVIEW_WINDOW, result);
        if (cv::waitKey(1) == 27)
            break;
    }

    cv::destroyAllWindows();

    return 0;
}