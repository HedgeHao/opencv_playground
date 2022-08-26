#include <iostream>
#include <opencv2/core/core.hpp>

class Config
{
public:
    int mode = 0; // 0: main, 1: calibration
    int calibration_rows = 9;
    int calibration_cols = 15;
    std::string params_file = "params.txt";
    cv::Matx33f K;
    cv::Vec<float, 5> k;
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

    void toString()
    {
        printf("[Arguments] >>>>>>>>>>>>>>>>>>>>>\n"
               "mode=%d\n"
               "calibration_rows=%d\n"
               "calibration_cols=%d\n"
               "params_file=%s\n"
               "[Arguments] <<<<<<<<<<<<<<<<<<<<\n",
               mode, calibration_rows, calibration_cols, params_file.c_str());
    }
};

std::string getNextArg(int argc, char **argv, int target)
{
    return target >= argc ? "" : std::string(argv[target]);
}

void parseArgument(int argc, char **argv, Config *config)
{
    int index = 1;
    bool printHelp = false;
    for (; index < argc; index++)
    {
        std::string arg = argv[index];
        if (arg == "-c" || arg == "--calibration")
        {
            config->mode = 1;
        }
        else if (arg == "-p" || arg == "--params")
        {
            config->params_file = getNextArg(argc, argv, ++index);
        }
        else
        {
            printHelp = true;
        }
    }

    if (printHelp)
    {
        printf("[Help]\n");
    }

    config->toString();
}