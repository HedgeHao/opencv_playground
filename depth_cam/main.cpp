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

rs2::frame depth_frame;

void onMouse(int event, int x, int y, int, void *)
{
    printf("x = %d, y = %d, depth = %.4f\n", x, y, depth_frame.as<rs2::depth_frame>().get_distance(x, y));
}

void calibration(rs2::pipeline &pipe, Config *config)
{
    std::vector<cv::Point2f> corners;
    cv::Mat frame;
    cv::Mat gray;
    while (true)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();
        frame = frame_to_mat(color_frame);
        cv::Size patternSize = cv::Size(config->calibration_cols, config->calibration_rows);

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        bool patternFound = cv::findChessboardCorners(gray, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        if (patternFound)
        {
            cv::drawChessboardCorners(frame, patternSize, corners, patternFound);
            cv::imshow(PREVIEW_WINDOW, frame);
            cv::waitKey(0);
            break;
        }
    }
}

int main(int argc, char **argv)
{
    Config *config = new Config();
    parseArgument(argc, argv, config);

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
    return 0;
}