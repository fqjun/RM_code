#ifndef RM_ARMORFITTED_1_H
#define RM_ARMORFITTED_1_H

#include "configure.h"
#include "rm_roi.h"
#include "serialport.h"
#include "kalmantest.h"



// 匹配灯条的结构体
struct matched_rect
{
    RotatedRect rect;
    float lr_rate;
    float angle_abs;
};

class RM_ArmorFitted
{
private:
    ArmorROI roi;
    matched_rect real_rect;
    RM_kalmanfilter kalman;

    Mat src_img;    //原图
    Mat gray_img;   //灰度图
    Mat hsv_img;    //hsv图
#ifdef NUMBER
    Mat bin_img;    //二值图
#endif
    Mat dst_img;    //输出图
    Mat lab_img;    //Lab
    Mat mask;       //mask
    Mat copy_img;   //copy_img
    Mat roi_img;    //ROI区域


    int  min_light_height= 10;// 板灯最小高度值
    int min_light_delta_w=12;// 左右灯柱在水平位置上的最大差值，像素单位
    int max_light_delta_w=450;// 左右灯柱在水平位置上的最小差值，像素单位
    int max_light_delta_v=50;// 左右灯柱在垂直位置上的最大差值，像素单位
    float max_lr_rate=2;// 左右灯柱的比例值 1.5
//------------Red----------------
    int g_RLmin = 4;
    int g_RAmin = 140;
    int g_RBmin = 20;

    int g_RLmax = 150;
    int g_RAmax = 206;
    int g_RBmax = 102;
//-----------Blue----------------
    int g_BLmin = 100;
    int g_BAmin = 80;
    int g_BBmin = 110;

    int g_BLmax = 200;
    int g_BAmax = 190;
    int g_BBmax = 210;
//------------Max----------------
    int g_max = 255;

    int model = 1;
    int model_select = 1;
    int depth = 1;


public:


#ifdef NUMBER
    ArmorROI roi_number;
#endif
    //丢失目标时的帧计数
    short int lose_target_cnt = 0;

    //是否丢失目标
    bool is_Lost_target = true;
    //上一帧是否有数据
    bool is_last_data_catch = false;

    //储存上一帧检测到的装甲的旋转矩形
    RotatedRect last_armor = RotatedRect();


    void imageProcessing(Mat frame,int my_color);
    void armorFitted();
    RotatedRect boundingRRect( cv::RotatedRect  left,  cv::RotatedRect  right);//拟合矩形函数
    RotatedRect boundingRRect_roi( cv::RotatedRect  left,  cv::RotatedRect  right,ArmorROI &roirect);//roi拟合矩形函数
#ifdef NUMBER
    void adaptiveThreshold( const cv::Mat& inImg, cv::Mat& outImg);//自适应阈值
#endif
};







#endif // RM_ARMORFITTED_H
