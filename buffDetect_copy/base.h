#ifndef BASE_H
#define BASE_H

#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

//标定XML路径
#define XML_PATH "/home/jun/workplace/qt/armor_test_V2_0/buffDetect/cameraParams_infantry4.xml"

//敌方颜色 1蓝 0红
#define COLOR 1

//二值化阈值 35
#define THRESHOLD 35

//model 1固定模型 0实时测距
#define MODEL 1

//buff-pre(buff_detect.cpp)
#define PRE_ANGLE 20
#define SMALL_LENTH_R 1.2
#define SMALL_PRE_ANGLE 20
#define BIG_LENTH_R 5

//buff-filter(buff_detect.cpp)
#define R 0.1

//buff-model尺寸(solve_pnp.cpp)
#define BULLET_SPEED 25//子弹射速
#define BUFF_BOTTOM_H 519//buff最底装甲板距离地面高度
#define ROBOT_H 400//枪口高度    现在是330~340
#define BUFF_ROBOT_Z 7200//枪口和buff的直线距离    6915.340249311
#define OFFSET_Y_BARREL_PTZ 27.69 //枪管和云台的高度差

#define W 300
#define H 170

//auto_control
// 能量机关自动控制项
//#define NO_FIRE   // 发现新目标射一发子弹
//#define NO_REPEAT_FIRE    // 没击打重复发
#define FIRE_CNT 30             // 越小响应越快
#define RESET_CNT 30            // 丢失目标复位计数 越小响应越快
#define REPEAT_FIRE_TIME 1000   // 重复发射时间，单位ｍｓ,可以修改，根据子弹飞行时间进行确认
#define RESET_ANGLE -10 // 1:-20 else: -10  // 复位绝对角度

#endif // BASE_H
