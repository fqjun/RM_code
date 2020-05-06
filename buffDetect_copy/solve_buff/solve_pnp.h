#ifndef SOLVE_PNP_H
#define SOLVE_PNP_H

#include"../base.h"

class Solve_Buff{
public:
    Solve_Buff();
    ~Solve_Buff(){}

    //调用解算函数
    void run_SolvePnp(Mat & srcImg, RotatedRect & rect, float buff_angle);

    float angle_x, angle_y, dist;

    float getBuffPitch(float dist, float tvec_y, float ballet_speed);

private:
    void vertex_Sort(RotatedRect & box);
    Mat camera_ptz(Mat & t);
    void get_Angle(const Mat & pos_in_ptz, float buff_angle);
    void draw_Coordinate(Mat & input);
    //标定数据
    string file_path = XML_PATH;
    Mat cameraMatrix, distCoeffs;
    Mat rvec = Mat::zeros(3, 3, CV_64FC1);
    Mat tvec = Mat::zeros(3, 1, CV_64FC1);

    vector<Point3f> armor3d;
    vector<Point2f> target2d;

    float ptz_camera_x = 0;
    float ptz_camera_y = 0;
    float ptz_camera_z = 0;
    float offset_y_barrel_ptz = 0;

    //装甲板单位mm
    float w = W;
    float h = H;

};


#endif // SOLVE_PNP_H
