#ifndef CONFIGURE_H
#define CONFIGURE_H

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <math.h>
#include <fstream>
#include "CameraApi.h"
#include <fcntl.h>  //文件控制定义
#include <termios.h>   //POSIX终端控制定义
#include <unistd.h>    //UNIX标准定义
#include <errno.h>     //ERROR数字定义
#include <sys/select.h>
/*---工业相机中使用到opencv2.0的 IplImage 需要包含此头文件 ---*/
#include "opencv2/imgproc/imgproc_c.h"

using namespace std;
using namespace cv;

#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))


//------------------------------调试开关-----------------------------------------
#define DEDUG
#define DEDUG_1

#define CAMERA
//#define VEDIO

//#define LR_rate
//#define Len

//#define NUMBER
#define TRACKBAR


//------------------------------------------------------------------------------

#ifdef CAMERA
#define CAPTURE_DEFULT 0//VideoCapture capture(CAPTURE_DEFULT)
/**
  @brief: 使用什么相机
  @param: 0 使用工业相机/电脑自带相机
  @param: 1/2/3 使用普通USB相机,对应你电脑上的usb口是第几个
*/
#endif

#ifdef VEDIO
#define CAPTURE_DEFULT "/home/jun/workplace/录像/armor_4.avi"//armor_2  大小装甲-红  步兵自旋-蓝  基地步兵-蓝
#endif

#define ISOPEN_INDUSTRY_CAPTURE 0
/**
  @brief: 是否使用工业相机
  @param: 0 使用工业相机
  @param: 1 使用普通USB相机
*/

#define CAMERA_EXPOSURETIME 500
#define CAMERA_RESOLUTION_COLS 640//1280
#define CAMERA_RESOLUTION_ROWS 480//800
#define CAMERA_RESOLUTION_COLS_FOV ((1280-CAMERA_RESOLUTION_COLS)*0.5)
#define CAMERA_RESOLUTION_ROWS_FOV ((1024-CAMERA_RESOLUTION_ROWS)*0.5)
/**
  @brief: 设置相机的分辨率
  @param: CAMERA_EXPOSURETIME   相机曝光时间
  @param: COLS                  为图像的宽度
  @param: ROWS                  为图像的高度
  @param: FOV                   为图像对应左上角的偏移值
  @note: 这部分相机文档中是写反的　x轴　和　y轴
         偏移值计算为 *** (相机最大分辨率 - 当前设置分辨率)/2 ***
*/
/*---------------------------------------------------*/

#define MY_COLOR 2
/**
  @brief: 选择己方阵营
  @param: 0     不限制颜色
  @param: 1     己方为红色
  @param: 2     己方为蓝色
  @param: 3     串口数据决定
  @note: 强制颜色模式
*/

/*---------------------------------------------------*/

/*---------------------------------------------------*/
//kalman各参数

#define IS_KF_PREDICT_ARMOR_OPEN 1
/**
  @brief: 是否启用卡尔曼
  @param: 0 不启用
  @param: 1 启用
*/

#define ANTI_RANGE 1.01//指数增长的底数
#define A 1.0e-6//给加速度a一个限定值(-A,A)之间
#define MNC 1e-10//测量协方差矩阵R，更大会有更慢的回归
#define DEAD_BAND 0
#define SIZE_X 960
#define SIZE_Y 480

//启用pid修正
#define PID
// pid修正参数
#define WIDTH 640
#define HEIGHT 480

#define KP 0.6
#define KI 0.02
#define KD 0.1

/*---------------------------------------------------*/
#define LOSE_CNT_MAX 4 //roi最大丢失帧数

#define SERIAL_IS_OPEN 1
/**
  @brief: 是否启用串口
  @param: 0 不启用
  @param: 1 启用
*/

/** 串口编号,1为不开,二开启USB0,3开启DAP,其他数字默认不开 **/
#define PORT_NUM 1

#define ALL_DEFAULT 1

#define SHOW_OUTPUT_IMG 1
/**
  @brief: 是否显示输出图像
  @param: 0     不显示
  @param: 1     显示
*/

#define COUT_FPS 1
/**
  @brief: 是否显示帧率
  @param: 0 不显示
  @param: 1 显示
*/

#define COUT_STATE 0
/**
  @brief: 是否显示状态信息
  @param: 0 不显示
  @param: 1 显示
*/

enum color{
    BLUE,
    ALL_COLOR,
    RED,
    /**
      @brief: 描述己方颜色信息
      @param: BLUE          己方为蓝色,敌方为红色
      @param: ALL_COLOR     无颜色信息，两种颜色都识别
      @param: RED           己方为红色,敌方为蓝色
    */
};

/**
 * @brief 装甲板区分
 * @note 默认为小装甲
 */
enum armor_size{
    little_armor = 1,
    big_armor = 2,
    armor_default = little_armor,
};

enum run_mode{
    DEFAULT_MODE,
    SUP_SHOOT,
    ENERGY_AGENCY,
    SENTRY_MODE,
    BASE_MODE,
    /**
      @brief: 描述运行模式信息
      @param: SUP_SHOOT         自瞄模式
      @param: ENERGY_AGENCY     神符模式
      @param: SENTRY_MODE       SENTRY模式
      @param: BASE_MODE         BASE模式
    */
};

enum Robot_ID{
    HERO = 1,
    ENGINEERING,
    INFANTRY,
    UAV = 6,
    SENTRY,
    /**
      @brief: 描述当前机器人ID信息
      @param: HERO          英雄
      @param: ENGINEERING   工程
      @param: INFANTRY      步兵
      @param: UAV           无人机
      @param: SENTRY        哨兵
    */
};


/**
 * @brief 大神符部分枚举
 */
enum big_chrysanthemum{
    //*quadrant_name
    first_quadrant = 1,
    second_quadrant = 2,
    third_quadrant = 3,
    fourth_quadrant = 4,
    origin = 5,

    //*change_follow_fixed_mode
    fixed_mode = 6,
    follow_mode = 7,

    //*send_mode
    send_none = 8,
    send_fixed_center = 9,
    send_shoot_point = 10,
    send_back_to_center = 11,

    //*execution_mode
    fixed_center = 12,
    shoot_point_quadrant = 13,
    Refresh_quadrant = 14,
};






#endif // CONFIGURE_H
