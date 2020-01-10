#ifndef RM_ROI_H
#define RM_ROI_H

#include "configure.h"

#define LOSE_CNT_MAX 4

struct ArmorROI
{
    RotatedRect last_armor_rect = RotatedRect();      // 上一次检测到装甲的旋转矩形
    Point tl = Point(0,0);                         // 截图区域的左上角的点
    int width = 0;                        // 截图区域的宽度
    int height = 0;                       // 截图区域的高度
    Mat ROI_img;                      // ROI区域


    //获得参数的函数
    void getParam(RotatedRect rect,Point point,int W,int H,Mat src){
        src.copyTo(ROI_img);
        last_armor_rect = rect;
        tl = point;
        width = W;
        height = H;
    }

    //重置结构体中的参数
    void resetParam(){
        last_armor_rect = RotatedRect();      // 上一次检测到装甲的旋转矩形
        tl = Point(0,0);                         // 截图区域的左上角的点
        width = 0;                        // 截图区域的宽度
        height = 0;
    }


};

ArmorROI setImg(Mat src,bool is_Lost_target,short int cnt,RotatedRect last_roi_rect);
ArmorROI setNumber(Mat src,bool is_Lost_target,short int cnt,RotatedRect last_roi_rect);

#endif // RM_ROI_H
