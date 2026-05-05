#include "armor_detector.hpp"
#include <iostream>

int main(int argc, char* argv[])
{
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
        1500, 0, 640, 0, 1500, 360, 0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    ArmorParams params;
    //蓝色
    params.blue_lower_hsv = cv::Scalar(100, 120, 120);
    params.blue_upper_hsv = cv::Scalar(130, 255, 255);
    //红色
    params.red_lower_hsv1 = cv::Scalar(0, 120, 120);
    params.red_upper_hsv1 = cv::Scalar(10, 255, 255);
    params.red_lower_hsv2 = cv::Scalar(170, 120, 120);
    params.red_upper_hsv2 = cv::Scalar(180, 255, 255);
    //形态学参数
    params.open_kernel_size = 3;
    params.close_kernel_size = 5;
    //灯条筛选
    params.min_light_area = 5.0f;
    params.max_light_area = 4000.0f;
    params.min_light_ratio = 0.1f;
    params.max_light_ratio = 10.0f;
    params.max_light_angle = 60.0f;
    //装甲板筛选
    params.max_center_y_diff = 50.0f;
    params.max_angle_diff = 25.0f;
    params.min_armor_ratio = 0.5f;
    params.max_armor_ratio = 10.0f;
    //装甲板物理尺寸
    params.armor_width = 140.0f;
    params.armor_height = 125.0f;
    //检测器
    ArmorDetector detector(params, camera_matrix, dist_coeffs);
    //默认蓝色
    std::string color = "blue";
    //查找--color red 或 --color blue
    for(int i = 1;i < argc - 1; i++)
    {
        if(std::string(argv[i]) == "--color")
        {
            color = argv[i + 1];
            break;
        }
    }
    detector.setColor(color);

    std::vector<std::string> filtered_args;
    for(int i = 1; i < argc; i++)
    {
        if(std::string(argv[i]) == "--color" && i + 1 < argc)
        {
            i++;
            continue;
        }
        filtered_args.push_back(argv[i]);
    }

    if(filtered_args.empty())
    {
        std::cout<<"用法：\n"
                 <<"  "<< argv[0] << "[--color red|blue] <图片路径>     检测单张图片\n"
                 <<"  "<< argv[0] << "[--color red|blue] <视频路径>     检测视频文件\n"
                 <<"  "<< argv[0] << " [--color red|blue] cam          使用摄像头\n"
                 <<"\n默认使用摄像头(蓝色)...\n";
        detector.detectVideo("0");
        return 0;
    }

    std::string arg = filtered_args[0];
    if(arg == "cam" || arg == "0")
    {
        detector.detectVideo("0");
        return 0;
    }
    //扩展名判断类型
    std::string ext;
    size_t dot_pos = arg.find_last_of('.');
    if(dot_pos != std::string::npos)
    {
        ext = arg.substr(dot_pos);
    }
    if(ext == ".jpg")
    {
        detector.detectImage(arg);
    }
    else if(ext == ".mp4")
    {

        detector.detectVideo(arg);
    }
    else
    {
        std::cout<<"位置类型，尝试作为视频打开"<<std::endl;
        detector.detectVideo(arg);

    }
    return 0;
}