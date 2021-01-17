#ifndef RM_KALMANFILTER_H
#define RM_KALMANFILTER_H

#include "configure.h"
#include "control/debug_control.h"

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

class RM_kalmanfilter
{
public:
    RM_kalmanfilter();
    ~RM_kalmanfilter();
    Point2f point_Predict(double g_runtime,Point2d current_point);

    //pid修正
    float point_dis(Point p1, Point p2);  // 两点距离
    Point pid_Control_predictor(Point predict, Point local);  // PID控制,返回的是一个修正后的点

private:
    Mat measurement_img;//测量矩阵
    cv::KalmanFilter kalman_img;
    Point2f last_point;
//    Point2f last_predic_point;
    double runtime=(1e-2) + 0.005666666f;
    double last_v=0;
    int n=2;

    //pid修正
    int predict_x;  // 预测位x
    int predict_y;  // 预测位y

    int last_x;  // 当前装甲板x
    int last_y;  // 当前装甲板y

    int true_x;  // 修正后的x
    int true_y;  // 修正后的y

};
/******************************************************************/
class KF_buff
{
public:
        KF_buff();
        ~KF_buff();
        Point2f Predict(/*Mat src,Point2f center_point,double r,*/double g_runtime, double current_angle,int direction_tmp, Point2f target_center, Point2f round_center);
        static inline Point calcPoint(Point2f center, double R, double angle)
        {
            return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
        }
private:
    cv::KalmanFilter kf_buff;
    double buff_last_angle;
    double buff_last_v;

    Mat buff_processNoise;
    Mat buff_measurement;
    Mat buff_state; /* (phi, delta_phi) */

    double buff_runtime=(1e-2) + 0.005666666f;
};





#endif // RM_KALMANFILTER_H
