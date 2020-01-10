#include "solve_pnp.h"

Solve_Buff::Solve_Buff(){
    //读取摄像头标定xml文件
    FileStorage fs(file_path, FileStorage::READ);

    //读取相机内参和畸变矩阵
    fs["camera-matrix"] >> cameraMatrix;
    fs["distortion"] >> distCoeffs;
    cout << cameraMatrix << endl;
    cout << distCoeffs << endl;

    armor3d.push_back(Point3f(-w/2, -h/2, 0));
    armor3d.push_back(Point3f(w/2, -h/2, 0));
    armor3d.push_back(Point3f(w/2, h/2, 0));
    armor3d.push_back(Point3f(-w/2, h/2, 0));

    cout << "RM_SolveAngle is readied" << endl;
}


void Solve_Buff::run_SolvePnp(Mat & srcImg, RotatedRect & rect, float buff_angle){

    vertex_Sort(rect);

    solvePnP(armor3d, target2d, cameraMatrix, distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);

    //draw_Coordinate(srcImg);

    Mat ptz = camera_ptz(tvec);
    //cout << ptz << "-----" << rect.center << endl;

    get_Angle(ptz, buff_angle);
}


void Solve_Buff::vertex_Sort(RotatedRect & box){
    Point2f vertex[4];
    Point2f lu, ld, ru, rd;

    box.points(vertex);

    sort(vertex, vertex + 4, [](const Point2f & p1, const Point2f & p2) { return p1.x < p2.x; });

    if (vertex[0].y < vertex[1].y){
        lu = vertex[0];
        ld = vertex[1];
    }
    else{
        lu = vertex[1];
        ld = vertex[0];
    }
    if (vertex[2].y < vertex[3].y)	{
        ru = vertex[2];
        rd = vertex[3];
    }
    else {
        ru = vertex[3];
        rd = vertex[2];
    }

    target2d.clear();
    target2d.push_back(lu);
    target2d.push_back(ru);
    target2d.push_back(rd);
    target2d.push_back(ld );
}

Mat Solve_Buff::camera_ptz(Mat & t){
    //设相机坐标系绕X轴你是逆时针旋转θ后与云台坐标系的各个轴向平行
    double theta = 0;/*-atan(static_cast<double>(ptz_camera_y + barrel_ptz_offset_y))/static_cast<double>(overlap_dist);*/
    double r_data[] = {1,0,0,0,cos(theta),sin(theta),0,-sin(theta),cos(theta)};
    //设相机坐标系的原点在云台坐标系中的坐标为(x0,y0,z0)
    double t_data[] = {static_cast<double>(ptz_camera_x),static_cast<double>(ptz_camera_y),static_cast<double>(ptz_camera_z)};
    Mat r_camera_ptz(3,3,CV_64FC1,r_data);
    Mat t_camera_ptz(3,1,CV_64FC1,t_data);

    Mat position_in_ptz = r_camera_ptz * t - t_camera_ptz;
    //cout << position_in_ptz << endl;
    return position_in_ptz;
}

void Solve_Buff::get_Angle(const Mat & pos_in_ptz, float buff_angle){
    //计算子弹下坠补偿
    const double *_xyz = (const double *)pos_in_ptz.data;
    //cout << "x:" << _xyz[0] << "   y:" << _xyz[1] << "   z:" << _xyz[2] << endl;

    //    double down_t = 0.0;
    //    if(BULLET_SPEED > 10e-3)
    //        down_t = _xyz[2] / BULLET_SPEED;

    //    double offset_gravity = 0.5 * 9.8 * down_t*down_t;
    double z = _xyz[2];
    if(MODEL == 1){
        //        float buff_bottom_h = 519;
        //        float robot_h = 400;
        //        float buff_robot_z = 7200;
        float buff_robot_y = BUFF_BOTTOM_H - ROBOT_H;
        float predict_buff_angle = buff_angle + PRE_ANGLE;
        if(predict_buff_angle > 360)
            predict_buff_angle = predict_buff_angle - 360.0f;
        float buff_h = 800*sin(predict_buff_angle *CV_PI/180)+800;
        float target_h = buff_robot_y + buff_h;
        float distance = sqrt(pow(target_h, 2) + pow(BUFF_ROBOT_Z, 2));
        z = distance;
        //cout << predict_buff_angle << endl;
        //cout << buff_h << endl;
    }
    double down_t = 0.0;
    if(BULLET_SPEED > 10e-3)
        down_t = z * 0.001 / BULLET_SPEED;

    double offset_gravity = 0.5 * 9.8 * down_t * down_t * 1000;
    //cout << offset_gravity << endl;
    double xyz[3] = {_xyz[0], _xyz[1] - offset_gravity, z};

    //计算角度
    double alpha = 0.0, theta = 0.0;
    alpha = asin(offset_y_barrel_ptz/sqrt(xyz[1]*xyz[1] + xyz[2]*xyz[2]));
    theta = atan(xyz[1]/xyz[2]);
    angle_y = -(theta-alpha);
    //cout << xyz[1] << endl;
    //    if(xyz[1] < 0){
    //        theta = atan(-xyz[1]/xyz[2]);
    //        angle_y = -(alpha+theta);  // camera coordinate
    //    }
    //    else if (xyz[1] < offset_y_barrel_ptz){
    //        theta = atan(xyz[1]/xyz[2]);
    //        angle_y = -(alpha-theta);  // camera coordinate
    //    }
    //    else{
    //        theta = atan(xyz[1]/xyz[2]);
    //        angle_y = (theta-alpha);   // camera coordinate
    //    }
    angle_x = atan2(xyz[0], xyz[2]);
    angle_x = angle_x * 180 / CV_PI;
    angle_y = angle_y * 180 / CV_PI;
    dist = xyz[2];

//    cout << "angle_x:" << angle_x << "     angle_y:" << angle_y << "    dist:" << dist <<endl;
}

void Solve_Buff::draw_Coordinate(Mat & input){
    vector<Point2f> reference_Img;
    vector<Point3f> reference_Obj;
    reference_Obj.push_back(Point3f(0.0, 0.0, 0.0));
    reference_Obj.push_back(Point3f(50, 0.0, 0.0));
    reference_Obj.push_back(Point3f(0.0, 50, 0.0));
    reference_Obj.push_back(Point3f(0.0, 0.0, 50));

    projectPoints(reference_Obj, rvec, tvec, cameraMatrix, distCoeffs, reference_Img);

    line(input, reference_Img[0], reference_Img[1], Scalar(0, 0, 255), 2);
    line(input, reference_Img[0], reference_Img[2], Scalar(0, 255, 0), 2);
    line(input, reference_Img[0], reference_Img[3], Scalar(255, 0, 0), 2);

    //imshow("outImg", input);
}

