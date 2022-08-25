#include <iostream>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define PREVIEW_WINDOW "Preview"
#define LINE_LENGTH 1.0f

void draw(cv::Mat &img, cv::Point2f corner, std::vector<cv::Point2f> imgpts)
{
    // std::cout << corner << imgpts << std::endl;
    cv::line(img, corner, imgpts[0], cv::Scalar(255, 0, 0), 5);
    cv::line(img, corner, imgpts[1], cv::Scalar(0, 255, 0), 5);
    cv::line(img, corner, imgpts[2], cv::Scalar(0, 0, 255), 5);
}

void drawBoxes(cv::Mat img, std::vector<cv::Point3f> imgpts)
{
}

int main(int argc, char **argv)
{
    cv::Matx33f K{1396.2198, 0, 959.5, 0, 1396.2198, 539.5, 0, 0, 1}; // intrinsic camera matrix
    cv::Vec<float, 5> k(0.0925378, -0.225531, 0, 0, 0);               // distortion coefficients

    cv::Size patternSize(16 - 1, 10 - 1);
    std::vector<cv::Point2f> corners;

    std::vector<cv::Point3f> objPoints;
    for (int i = 0; i < 15; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            objPoints.push_back(cv::Point3f((float)i, (float)j, 0.0f));
        }
    }
    std::vector<cv::Point3f> axis{{LINE_LENGTH, 0.0f, 0.0f}, {0.0f, LINE_LENGTH, 0.0f}, {0.0f, 0.0f, -LINE_LENGTH}};
    std::vector<cv::Point3f> axisBoxes{cv::Point3f(0, 0, 0), cv::Point3f(0, LINE_LENGTH, 0), cv::Point3f(LINE_LENGTH, LINE_LENGTH, 0), cv::Point3f(LINE_LENGTH, 0, 0), cv::Point3f(0, 0, -LINE_LENGTH), cv::Point3f(0, LINE_LENGTH, -LINE_LENGTH), cv::Point3f(LINE_LENGTH, LINE_LENGTH, -LINE_LENGTH), cv::Point3f(LINE_LENGTH, 0, -LINE_LENGTH)};

    cv::Mat frame;
    cv::Mat gray;
    cv::VideoCapture cap(std::stoi(argv[1]));
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    std::vector<cv::Point2f> imagePoints{};

    char c;
    while (true)
    {
        bool ret = cap.read(frame);
        if (!ret)
            continue;

        cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
        bool patternFound = cv::findChessboardCorners(gray, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
        if (patternFound)
        {
            cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            cv::solvePnP(objPoints, corners, K, k, rvec, tvec);

            cv::projectPoints(axis, rvec, tvec, K, k, imagePoints);
            draw(frame, corners[5], imagePoints);

            // cv::projectPoints(axisBoxes, rvec, tvec, K, k, imagePoints);
            // drawBoxes(frame, corners[0], imagePoints);
        }

        cv::imshow(PREVIEW_WINDOW, frame);
        c = cv::waitKey(1);
        if (c == 27)
            break;
    }

    cv::destroyAllWindows();

    return 0;
}