#ifndef ARMOR_DETECTOR_H
#define ARMOR_DETECTOR_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

//灯条
struct LightBar 
{
    cv::RotatedRect rect;//最小外接旋转矩阵
    float length;//灯条长度
    float width;//灯条宽度
    float angle;//灯条倾斜角
    cv::Point2f center;//中心点
    cv::Point2f top;//上端点
    cv::Point2f bottom;//下端点
    float area;//轮廓面积
};

//装甲板
struct Armor
{
    LightBar left_light;//左灯条
    LightBar right_light;//右灯条
    std::vector<cv::Point2f> image_points;//装甲板四角点
    cv::Point2f center;//装甲板中心
    float width_ratio;//装甲板宽高比
};
//装甲板检测
struct ArmorParams
{
    //HSV
    cv::Scalar blue_lower_hsv = cv::Scalar(100, 120, 120);
    cv::Scalar blue_upper_hsv = cv::Scalar(130, 255, 255);
    cv::Scalar red_lower_hsv1 = cv::Scalar(0, 120, 120);
    cv::Scalar red_upper_hsv1 = cv::Scalar(10, 255, 255);
    cv::Scalar red_lower_hsv2 = cv::Scalar(170, 120, 120);
    cv::Scalar red_upper_hsv2 = cv::Scalar(180, 255, 255);

    cv::Scalar lower_hsv1;
    cv::Scalar upper_hsv1;
    cv::Scalar lower_hsv2;
    cv::Scalar upper_hsv2;
    bool use_dual_range = false;


    //核
    int open_kernel_size = 3;
    int close_kernel_size = 5;
    //灯条筛选
    float min_light_area = 5.0f;
    float max_light_area = 5000.0f;
    float min_light_ratio = 1.5f;
    float max_light_ratio = 15.0f;
    float max_light_angle = 30.0f;//最大倾角
    //装甲板筛选
    float max_center_y_diff = 40.0f;//中心最大y
    float max_angle_diff = 10.0f;//左右灯条角度最大差
    float min_armor_ratio = 0.5f;//宽高比最小
    float max_armor_ratio = 10.0f;
    //装甲板物理尺寸
    float armor_width = 140.0f;
    float armor_height = 125.0f;
};

class ArmorDetector
{
    public:
        ArmorDetector(const ArmorParams& params, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs);
        std::vector<Armor> detect(const cv::Mat& frame);
        cv::Mat getDebugImage();
        void detectImage(const std::string& image_path);//单张图片
        void detectVideo(const std::string& video_path);//0,cam为摄像头
        void setColor(const std::string& color);//red或blue

    private:
        ArmorParams params_;
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;

        cv::Mat gray_;
        cv::Mat blurred_;
        cv::Mat hsv_;
        cv::Mat binary_;
        cv::Mat opened_;
        cv::Mat closed_;
        cv::Mat debug_image_;

        void preprocess(const cv::Mat& frame);
        std::vector<LightBar> findLightBars();
        std::vector<Armor> matchArmors(const std::vector<LightBar>& light_bars);
        bool isValidLightBar(const LightBar& light);
        void getLightBarEndpoints(LightBar& light);
        cv::Mat solveArmorPose(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec);
};
#endif
