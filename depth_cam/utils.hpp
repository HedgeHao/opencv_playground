#include <fstream>

void generateChessboardCornerPoints3D(std::vector<cv::Point3f> &objPoints, int cols, int rows)
{
    objPoints.clear();
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            objPoints.push_back(cv::Point3f(j, i, 0));
        }
    }
}

void read_camera_params(std::string file, cv::Matx33f &K, cv::Vec<float, 5> &k)
{
    float K1, K2, K3, K4, K5, K6, K7, K8, K9;
    float k1, k2, k3, k4, k5;

    std::ifstream infile(file);
    infile >> K1 >> K2 >> K3 >> K4 >> K5 >> K6 >> K7 >> K8 >> K9;
    infile >> k1 >> k2 >> k3 >> k4 >> k5;

    K = cv::Matx33f{K1, K2, K3, K4, K5, K6, K7, K8, K9};
    k = cv::Vec<float, 5>{k1, k2, k3, k4, k5};

    std::cout << "K = " << K << "\n"
              << "k = " << k << std::endl;
}

cv::Mat frame_to_mat(const rs2::frame &f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void *)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void *)f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void *)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void *)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void *)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}