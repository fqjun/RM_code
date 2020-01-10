#ifndef BASE_H
#define BASE_H

#include "configure.h"

//标定XML路径
#define XML_PATH "/home/jun/workplace/qt/armor_test_V2_0/buffDetect/cameraParams_infantry4.xml"

//敌方颜色 1蓝 0红
#define COLOR 1

//二值化阈值
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
#define ROBOT_H 400//枪口高度
#define BUFF_ROBOT_Z 7200//枪口和buff的直线距离

#define W 300
#define H 170


#endif // BASE_H
