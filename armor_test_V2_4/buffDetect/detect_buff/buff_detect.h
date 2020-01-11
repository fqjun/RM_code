#ifndef BUFF_DETECT_H
#define BUFF_DETECT_H

#include "buffDetect/base.h"
#include "buffDetect/solve_buff/solve_pnp.h"

typedef enum{UNKOWN,INACTION,ACTION}ObjectType;
class Object
{
public:
    Object(){}
    ~Object(){}
    void smallUpdate_Order(); // 更新能量机关装甲板的绝对位置
    void bigUpdate_Order(); //更新能量机关叶片的绝对位置
    void knowYour_Self(Mat &img);    //判断能量机关扇叶的状态（激活　未激活）

    RotatedRect small_rect_;    // 能量机关扇叶内轮廓
    RotatedRect big_rect_;  // 能量机关扇叶外轮廓
    vector<Point2f> points_2d_; //内轮廓装点集
    vector<Point2f> big_points_2d_; //叶片点集
    float angle_ = 0; //待激活装甲板 0~360度
    float diff_angle = 0; //待激活装甲板和叶片的相对角度（判断是否垂直）
    int type_ = UNKOWN; //是否激活种类初始化
};

class BuffDetector
{
public:
    BuffDetector() {solve_buff = Solve_Buff();} //solve.cpp的构造函数
    ~BuffDetector(){}
    int buffDetect_Task(Mat & frame); //主逻辑函数
    float angle_x, angle_y, dist; //输出数据(角度，距离)
    Point2f pre_center;//预测识别坐标
    Point2f target_center;//真实识别坐标


private://类的声明
    Solve_Buff solve_buff;
    Object object;
    Object final_target;
    Object object_tmp;

private:
    void imageProcess(Mat & frame); //预处理
    bool findTarget(Mat & frame);   //寻找叶片以及待激活装甲板
    bool findCenter_R(Mat & bin_img, Mat &frame); //寻找R
    int getState(); //能量机关顺逆时针滤波函数

private://Object object_tmp新类，用于装清洗出来的新数据 和一些需要公用的数据
     vector<Point2f> points_2d;
     //vector<Point2f> big_points_2d;
//     Point2f target_center;
     Point2f roi_center;
     Point2f circle_center;
//     Point2f pre_center;
     Mat bin_Img;
     RotatedRect solve_rect;

private://能量机关顺逆时针判断
    float buff_angle_ = 0;
    float diff_angle_ = 0;
    float last_angle = 0;
    float d_angle_ = 0;
    int find_cnt_ = 0;
    int direction_tmp_ = 0;

};

#endif // BUFF_DETECT_H
double pointDistance(Point2f & p1, Point2f & p2);//计算两点之间距离
int getRect_Intensity(const Mat &frame, Rect rect);//创建两小roi，判断叶片是否为未激活状态
