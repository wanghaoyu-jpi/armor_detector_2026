#include "armor_detector.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
//构造函数
ArmorDetector::ArmorDetector(const ArmorParams& params, const cv::Mat& camera_matrix, 
                             const cv::Mat& dist_coeffs)
    : params_(params)
    , camera_matrix_(camera_matrix.clone())
    , dist_coeffs_(dist_coeffs.clone())
{}


//预处理颜色分割
void ArmorDetector::preprocess(const cv::Mat& frame)
{
    //高斯滤波
    cv::GaussianBlur(frame, blurred_, cv::Size(3, 3), 0);
    cv::cvtColor(blurred_, hsv_, cv::COLOR_BGR2HSV);
    //第一段颜色分割
    cv::inRange(hsv_, params_.lower_hsv1, params_.upper_hsv1, binary_);

    if(params_.use_dual_range)
    {
        cv::Mat binary2;
        cv::inRange(hsv_, params_.lower_hsv2, params_.upper_hsv2, binary2);
        cv::bitwise_or(binary_, binary2, binary_);

    }

    //形态学操作
    cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(params_.open_kernel_size, params_.open_kernel_size));
    cv::morphologyEx(binary_, opened_, cv::MORPH_OPEN, open_kernel);
    cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(params_.close_kernel_size, params_.close_kernel_size));
    cv::morphologyEx(opened_, closed_, cv::MORPH_CLOSE, close_kernel);

}


//检测灯条
std::vector<LightBar> ArmorDetector::findLightBars()
{
    std::vector<LightBar> light_bars;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(closed_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for(const auto& contour : contours)
    {
        float area = (float)cv::contourArea(contour);
        if(area < params_.min_light_area || area > params_.max_light_area)
        {
            std::cout<<"[灯条检测] 淘汰，面积过大或过小"<<area<<std::endl;
            continue;
            
        }
        //最小外接旋转矩形
        cv::RotatedRect rect = cv::minAreaRect(contour);

        LightBar light;
        light.rect = rect;
        light.area = area;
        light.center = rect.center;

        light.width = std::min(rect.size.width, rect.size.height);
        light.length = std::max(rect.size.width, rect.size.height);

        if(light.width < 1e-3f)
        {
            std::cout<<"[灯条检测] 淘汰，灯条太窄"<<light.width<<std::endl;
            continue; 
        }
        float ratio = light.length / light.width;
        //倾斜角
        light.angle = rect.angle;
        if(rect.size.width > rect.size.height)
        {
            light.angle += 90.0f;
        }
        if(!isValidLightBar(light))
        {
            std::cout<<"[灯条检测] 淘汰，灯条无效"<<std::endl;
            continue;
        }

        getLightBarEndpoints(light);
        light_bars.push_back(light);
    }
    std::sort(light_bars.begin(), light_bars.end(), [](const LightBar& a, const LightBar& b)
    {
        return a.center.x < b.center.x;
    });
    return light_bars;
}



//灯条有效性判断
bool ArmorDetector::isValidLightBar(const LightBar& light)
{
    float ratio = light.length / light.width;//长宽比筛选
    if(ratio < params_.min_light_ratio || ratio > params_.max_light_ratio)
    {
        return false;
    }
    float abs_angle = std::abs(light.angle);//倾斜角筛选
    float angle_from_vertical = std::abs(90.0f - abs_angle);
    if(angle_from_vertical > params_.max_light_angle && angle_from_vertical < (90.0f - params_.max_light_angle))
    {
        return false;
    }
    return true;
}



//获取灯条端点
void ArmorDetector::getLightBarEndpoints(LightBar& light)
{
    cv::Point2f vertices[4];//四个角点
    light.rect.points(vertices);

    std::sort(vertices, vertices + 4, [](const cv::Point2f& a, const cv::Point2f& b)
    {
        return a.y < b.y;
    });
    light.top = (vertices[0] + vertices[1]) * 0.5f;
    light.bottom = (vertices[2] + vertices[3]) * 0.5f;
    if(light.top.y > light.bottom.y)
    {
        std::swap(light.top, light.bottom);
    }
   
}



//灯条匹配装甲板
std::vector<Armor> ArmorDetector::matchArmors(const std::vector<LightBar>& light_bars)
{
    std::vector<Armor> armors;
    if(light_bars.size() < 2)
    {
        std::cout<<"[配对]灯条不足两个，跳过配对"<<std::endl;
        return armors;
    }
    //遍历所有灯条两两配对
    std::cout<<"[配对] 开始配对，共"<<light_bars.size()<<"个灯条"<<std::endl;
    for(size_t i = 0; i < light_bars.size(); i++)
    {
        for(size_t j = i + 1; j < light_bars.size(); j++)
        {
            const LightBar& left = light_bars[i];//x较小的是左灯条
            const LightBar& right = light_bars[j];
            std::cout<<"[配对] 灯条"<<i<<"&灯条"<<j<<std::endl;
            //中心y坐标
            float center_y_diff = std::abs(left.center.y - right.center.y);
            if(center_y_diff > params_.max_center_y_diff)
            {
                std::cout<<"淘汰,y差:"<<center_y_diff<<std::endl;
                continue;
            }
            //角度差
            float angle_diff = std::abs(left.angle - right.angle);
            if(angle_diff > params_.max_angle_diff)
            {
                std::cout<<"淘汰,角度差:"<<angle_diff<<std::endl;
                continue;
            }
            //高度比
            float height_ratio = std::min(left.length, right.length) / std::max(left.length, right.length);
            if(height_ratio < 0.3f)
            {
                std::cout<<"淘汰,高度比:"<<height_ratio<<std::endl;
                continue;
            }
            //装甲板宽高比
            float armor_width = std::abs(left.center.x - right.center.x);
            float avg_height = (left.length + right.length) / 2.0f;
            float armor_ratio = armor_width / avg_height;
            if(armor_ratio < params_.min_armor_ratio || armor_ratio > params_.max_armor_ratio)
            {
                std::cout<<"淘汰, 装甲板宽高比:"<<armor_ratio<<std::endl;
                continue;
            }
            //灯条间距
            float light_x_dist = right.center.x - left.center.x;
            if(light_x_dist < avg_height * 0.5f || light_x_dist > avg_height* 8.0f)
            {
                std::cout<<"淘汰,灯条间距:"<<light_x_dist<<std::endl;
                continue;
            }
            std::cout<<"[配对]灯条"<<i<<"&灯条"<<j<<"配对成功"<<std::endl;
            //构建装甲板
            Armor armor;
            armor.left_light = left;
            armor.right_light = right;
            armor.image_points = {left.top, right.top, right.bottom, left.bottom};
            //装甲板中心
            armor.center = (left.center + right.center) * 0.5f;
            armor.width_ratio = armor_ratio;
            

            armors.push_back(armor);

        }
    }
    std::cout<<"[配对]完成，共匹配"<<armors.size() <<"个装甲板"<<std::endl;
    return armors;
}



//PnP
cv::Mat ArmorDetector::solveArmorPose(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec)
{
    if(armor.image_points.size() != 4)
    {
        return cv::Mat();
    }
    float half_w = params_.armor_width / 2.0f;
    float half_h = params_.armor_height / 2.0f;
    
    std::vector<cv::Point3f> object_points = 
    {
        cv::Point3f(-half_w, -half_h, 0),
        cv::Point3f(half_w, -half_h, 0),
        cv::Point3f(half_w, half_h, 0),
        cv::Point3f(-half_w, half_h, 0)

    };
    cv::solvePnP(object_points, armor.image_points, camera_matrix_, dist_coeffs_, rvec, tvec);
    cv::Mat rot_mat;
    cv::Rodrigues(rvec, rot_mat);
    return rot_mat;
}
cv::Mat ArmorDetector::getDebugImage()
{
    return debug_image_;
}




//测试函数
std::vector<Armor> ArmorDetector::detect(const cv::Mat& frame)
{
    std::vector<Armor> armors;
    if(frame.empty())
    {
        return armors;
    }
    //预处理
    preprocess(frame);
    //找灯条
    std::vector<LightBar> light_bars = findLightBars();
    //匹配装甲板
    armors = matchArmors(light_bars);

    debug_image_ = frame.clone();
    //绘制灯条并命名
    for (size_t idx = 0; idx < light_bars.size(); idx++)
    {
        const LightBar& light = light_bars[idx];

    
        cv::Point2f vertices[4];
        light.rect.points(vertices);
        for (int i = 0; i < 4; i++)
        {
            cv::line(debug_image_, vertices[i], vertices[(i + 1) % 4],
                     cv::Scalar(0, 255, 0), 2);
        }
        cv::line(debug_image_, light.top, light.bottom, cv::Scalar(255, 0, 0), 2);
        cv::circle(debug_image_, light.center, 4, cv::Scalar(0, 255, 0), -1);
        cv::circle(debug_image_, light.top, 3, cv::Scalar(0, 0, 255), -1);
        cv::circle(debug_image_, light.bottom, 3, cv::Scalar(255, 0, 0), -1);

    
        cv::putText(debug_image_,
                    std::to_string(idx),                     // 序号转字符串
                    light.center + cv::Point2f(10, -10),     // 显示在中心点右上方
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(0, 255, 255),                 // 黄色
                    2);
    }
    //绘制装甲板
    for(const auto& armor : armors)
    {
        if(armor.image_points.size() == 4)
        {
            for(int i = 0; i < 4; i++)
            {
                cv::line(debug_image_, armor.image_points[i], armor.image_points[(i + 1) % 4], cv::Scalar(0, 0, 255), 3);

            }
            cv::circle(debug_image_, armor.center, 5, cv::Scalar(0, 255, 255), -1);
            //装甲板中心坐标
            cv::putText(debug_image_, "(" + std::to_string((int)armor.center.x) + "," + 
                        std::to_string((int)armor.center.y) + ")", armor.center + cv::Point2f(10, 10), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
        }
        cv::Mat rvec, tvec;
        cv::Mat rot_mat = solveArmorPose(armor, rvec, tvec);
        if(!rot_mat.empty())
        {
            //打印距离
            float distance = std::sqrt(tvec.at<double>(0) * tvec.at<double>(0) +
                                       tvec.at<double>(1) * tvec.at<double>(1) +
                                       tvec.at<double>(2) * tvec.at<double>(2));
            std::cout<<"装甲板距离："<<distance / 1000.0 << "m" <<std::endl;
        }
    }
    return armors;
}



//检测单张图片
void ArmorDetector::detectImage(const std::string& image_path)
{
    cv::Mat frame = cv::imread(image_path);
    if(frame.empty())
    {
        std::cout<<"无法读取图片"<<image_path<<std::endl;
        return;
    }
    std::cout<<"图片尺寸："<<frame.cols<<"x"<<frame.rows<<std::endl;
    
    

    std::vector<Armor> armors = detect(frame);//检测
    
    //打印
    std::cout<<"检测到"<<armors.size()<<"个装甲板" <<std::endl;
    for(size_t i = 0;i < armors.size();i++)
    {
        std::cout<<"装甲板"<<i<<"中心：（"<<armors[i].center.x<<","<<armors[i].center.y<<")"<<std::endl;

    }
    //显示结果
    cv::Mat debug = getDebugImage();
    cv::imshow("Armor Detection - Image", debug);
    std::cout<<"按任意按键关闭..."<<std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();

    //保存
    std::string output_path = "result_" + image_path.substr(image_path.find_last_of("/\\") + 1);
    cv::imwrite(output_path, debug);
    std::cout << "结果保存到："<<output_path<<std::endl;

}



//检测视频
void ArmorDetector::detectVideo(const std::string& video_path)
{
    cv::VideoCapture cap;
    if(video_path == "0" || video_path == "cam")
    {
        cap.open(0);
        std::cout<<"使用摄像头"<<std::endl;
    }
    else
    {
        cap.open(video_path);
        std::cout<<"视频文件："<<video_path<<std::endl;

    }

    if(!cap.isOpened())
    {
        std::cout<<"无法打开视频："<<video_path<<std::endl;
        return;
    }
    int frame_width = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int frame_height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(cv::CAP_PROP_FPS);
    std::cout<<"分辨率："<<frame_width<<"x"<<frame_height<<",FPS"<<fps<<std::endl;

    cv::Mat frame;
    int frame_count = 0;

    while(true)
    {
        cap>>frame;
        if(frame.empty())
        {
            std::cout<<"视频结束，共处理"<<frame_count<<"帧"<<std::endl;
            break;
        }

        frame_count++;
        std::vector<Armor> armors = detect(frame);
        if(frame_count % 30 == 0)
        {
            std::cout<<"帧"<<frame_count;
            if(!armors.empty())
            {
                std::cout<<"| 装甲板："<<armors.size()<<"个";
                for(size_t i = 0;i<armors.size();i++)
                {
                    std::cout<<"["<<i<<"]:("<<(int)armors[i].center.x<<","<<(int)armors[i].center.y<<")";
                }
            }
            std::cout<<std::endl;

        }
        cv::Mat debug = getDebugImage();
        cv::putText(debug,"Frame:" + std::to_string(frame_count), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                    0.8, cv::Scalar(0, 255, 0),2);
        cv::imshow("Armor Detection - Video", debug);
        char key = (char)cv::waitKey(1);
        if(key == 27)//esc
        {
            std::cout<<"终止"<<std::endl;
            break;
        }
        else if (key == ' ')
        {
            std::cout<<"暂停中，按空格继续..."<<std::endl;
            while(true)
            {
                char key2 = (char)cv::waitKey(0);
                if(key2 == ' ')break;
                if(key2 == 27) goto exit_loop;
            }
            
        }
        
    }
    exit_loop:
    cap.release();
    cv::destroyAllWindows();
}
//设置颜色，默认为蓝色
void ArmorDetector::setColor(const std::string& color)
{
    if(color == "red")
    {
        params_.lower_hsv1 = params_.red_lower_hsv1;
        params_.upper_hsv1 = params_.red_upper_hsv1;
        params_.lower_hsv2 = params_.red_lower_hsv2;
        params_.upper_hsv2 = params_.red_upper_hsv2;
        params_.use_dual_range = true;
        std::cout<<"检测颜色:红色"<<std::endl;
    }else 
    {
        params_.lower_hsv1 = params_.blue_lower_hsv;
        params_.upper_hsv1 = params_.blue_upper_hsv;
        params_.use_dual_range = false;
        std::cout <<"检测颜色:蓝色"<<std::endl;
    }
    
}







