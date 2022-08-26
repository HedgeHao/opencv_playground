#include <iostream>
#include <vector>
#include <librealsense2/rs.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "utils.hpp"
#include "parse_arg.hpp"

#define PREVIEW_WINDOW "Preview"
#define PLANE_HEIGHT 2.0f

const cv::TermCriteria CRITERIA(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
const std::vector<cv::Point3f> axis{{PLANE_HEIGHT, 0.0f, 0.0f}, {0.0f, PLANE_HEIGHT, 0.0f}, {0.0f, 0.0f, -PLANE_HEIGHT}};

rs2::frame depth_frame;

void onMouse(int event, int x, int y, int, void *)
{
    printf("x = %d, y = %d, depth = %.4f\n", x, y, depth_frame.as<rs2::depth_frame>().get_distance(x, y));
}

void calibration(rs2::pipeline &pipe, Config *config)
{
    std::vector<cv::Point3f> plane = {
        {0, 0, PLANE_HEIGHT},
        {(float)config->calibration_cols, 0, PLANE_HEIGHT},
        {0, (float)config->calibration_rows, PLANE_HEIGHT},
        {(float)config->calibration_cols, (float)config->calibration_rows, PLANE_HEIGHT},
    };
    std::vector<cv::Point3f> objPoints;
    generateChessboardCornerPoints3D(objPoints, config->calibration_cols, config->calibration_rows);

    std::vector<cv::Point2f> imagePoints{};
    std::vector<cv::Point2f> corners;
    cv::Mat frame;
    cv::Mat gray;
    while (true)
    {
        rs2::frameset frames = pipe.wait_for_frames(3000);
        rs2::video_frame color_frame = frames.get_color_frame();
        frame = frame_to_mat(color_frame);
        cv::Size patternSize = cv::Size(config->calibration_cols, config->calibration_rows);

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        bool patternFound = cv::findChessboardCorners(gray, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        if (patternFound)
        {
            // cv::drawChessboardCorners(frame, patternSize, corners, patternFound);

            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), CRITERIA);
            cv::solvePnP(objPoints, corners, config->K, config->k, config->rvec, config->tvec);
            cv::projectPoints(plane, config->rvec, config->tvec, config->K, config->k, imagePoints);

            cv::line(frame, corners[0], imagePoints[0], cv::Scalar(0, 0, 255), 2);
            cv::line(frame, corners[14], imagePoints[1], cv::Scalar(0, 0, 255), 2);
            cv::line(frame, corners[120], imagePoints[2], cv::Scalar(0, 0, 255), 2);
            cv::line(frame, corners[corners.size() - 1], imagePoints[3], cv::Scalar(0, 0, 255), 2);

            cv::line(frame, imagePoints[0], imagePoints[1], cv::Scalar(0, 255, 0), 2);
            cv::line(frame, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 2);
            cv::line(frame, imagePoints[3], imagePoints[1], cv::Scalar(0, 255, 0), 2);
            cv::line(frame, imagePoints[3], imagePoints[2], cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow(PREVIEW_WINDOW, frame);
        if (cv::waitKey(1) == 27)
            break;
    }
}

int main(int argc, char **argv)
{

    Config *config = new Config();
    parseArgument(argc, argv, config);
    read_camera_params(config->params_file, config->K, config->k);

    rs2::pipeline pipe;
    pipe.start();

    if (config->mode == 1)
    {
        calibration(pipe, config);
    }

    cv::namedWindow(PREVIEW_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(PREVIEW_WINDOW, onMouse);

    cv::Mat frame;
    char c;
    while (true)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();
        depth_frame = frames.get_depth_frame();
        frame = frame_to_mat(color_frame);
        cv::imshow(PREVIEW_WINDOW, frame);
        c = cv::waitKey(1);
        if (c == 27)
        {
            break;
        }
    }

    cv::destroyAllWindows();

    pipe.stop();

    return 0;
}