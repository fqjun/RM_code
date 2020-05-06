#include "buff_detect.h"

void BuffDetector::imageProcess(Mat & frame){

    Mat gauss_img;
    GaussianBlur(frame, gauss_img, Size(3, 3), 0);//高斯滤波
    points_2d.clear();
    vector<Mat> bgr;

    split(gauss_img, bgr);
    if(COLOR == 0){
        subtract(bgr[2], bgr[0], gauss_img);
    }
    else{
        subtract(bgr[0], bgr[2], gauss_img);
    }

    double th = threshold(gauss_img, bin_Img, THRESHOLD, 255,  0);

    //    if(th-10 > 0)
    //        threshold(gaussImg, binImg, th-15, 255,  0);

    dilate(bin_Img, bin_Img, getStructuringElement(MORPH_RECT, Size(3,3)));
    //    imshow("bin", bin_Img);
    //    cout << "th:" << th << endl;
}

bool BuffDetector::findTarget(Mat & frame){

    vector<Object> vec_target;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    //    imshow("bin_test",bin_Img);
    findContours(bin_Img, contours, hierarchy, 2, CHAIN_APPROX_NONE);
    for(int i = 0; i < (int)contours.size(); ++i){
        // 用于寻找小轮廓，没有父轮廓的跳过, 以及不满足6点拟合椭圆
        if(hierarchy[i][3]<0 || contours[i].size() < 6 || contours[static_cast<uint>(hierarchy[i][3])].size() < 6)
            continue;

        // 小轮廓面积条件
        double small_rect_area = contourArea(contours[i]);
        double small_rect_length = arcLength(contours[i],true);
        if(small_rect_length < 10)
            continue;
        if(small_rect_area < 200 || small_rect_area > 2000)
            continue;
        // 大轮廓面积条件
        double big_rect_area = contourArea(contours[static_cast<uint>(hierarchy[i][3])]);
        double big_rect_length = arcLength(contours[static_cast<uint>(hierarchy[i][3])],true);
        if(big_rect_area < 300 || big_rect_area > 1e4)
            continue;
        if(big_rect_length < 50)
            continue;

        //Object object;

        object.small_rect_=fitEllipse(contours[i]);
        object.big_rect_ = fitEllipse(contours[static_cast<uint>(hierarchy[i][3])]);

        Point2f small_point_tmp[4];
        object.small_rect_.points(small_point_tmp);
        Point2f big_point_tmp[4];
        object.big_rect_.points(big_point_tmp);

        //组合符合条件的装甲板和叶片
        object.diff_angle =fabsf(object.big_rect_.angle-object.small_rect_.angle);
        if(object.diff_angle<100 && object.diff_angle>80){
            float small_rect_size_ratio;
            if(object.small_rect_.size.width > object.small_rect_.size.height){
                small_rect_size_ratio = object.small_rect_.size.width/object.small_rect_.size.height;
            }else {
                small_rect_size_ratio = object.small_rect_.size.height/object.small_rect_.size.width;
            }
            //cout << big_rect_area << "  " << small_rect_area << endl;
            double area_ratio = (double)object.small_rect_.boundingRect().area()/(double)object.big_rect_.boundingRect().area();
            //cout << area_ratio << endl;
            object.type_ = UNKOWN;
            //再次清洗目标找出叶片
            if(small_rect_size_ratio>1 && small_rect_size_ratio<3 && area_ratio>0.08f && area_ratio<0.25f){
                for(int k=0;k<4;k++){
                                    line(frame, small_point_tmp[k],small_point_tmp[(k+1)%4], Scalar(0, 0, 255), 1);
                                //line(frame, big_point_tmp[k],big_point_tmp[(k+1)%4], Scalar(0, 0, 255), 1);
                                }
                object.type_= ACTION;
                //直接找出未激活状态目标（待优化）
                if(/*small_rect_area*9>big_rect_area && small_rect_area*3<big_rect_area*/0){
                    object.type_ = INACTION;
                    object.smallUpdate_Order();
                    //object.bigUpdateOrder();
                    for(int k=0;k<4;k++){
                        // line(frame, small_point_tmp[k],small_point_tmp[(k+1)%4], Scalar(0, 0, 255), 3);
                        //line(img, big_point_tmp[k],big_point_tmp[(k+1)%4], Scalar(0, 0, 255), 1);
                    }
                }
                else {//未能找出未激活目标,将识别的激活目标逐个筛选出
                    object.smallUpdate_Order();//更新装甲板的四个角的编号
                    object.knowYour_Self(bin_Img);//用roi判断该装甲板是否已激活
                    if(object.type_ == INACTION){//若筛选出
                        for(int k=0;k<4;k++){
                             line(frame, small_point_tmp[k],small_point_tmp[(k+1)%4], Scalar(0, 255, 0), 3);
                            //line(img, big_point_tmp[k],big_point_tmp[(k+1)%4], Scalar(0, 0, 255), 1);
                        }
                        //object.bigUpdate_Order();
                    }
                    else {//未能筛选出未激活状态目标
                                //                        for(int k=0;k<4;k++){
                                // line(img, small_point_tmp[k],small_point_tmp[(k+1)%4], Scalar(0, 255, 255), 1);
                                // line(img, big_point_tmp[k],big_point_tmp[(k+1)%4], Scalar(0, 0, 255), 1);
                                //   }
                    }
                }
                //所有识别目标装进容器,无论是否是激活状态
                vec_target.push_back(object);
            }
        }
    }
    //cout << vec_target.size() << endl;
    //    Object final_target;
    bool is_target = false;
    for(int i2 = 0; i2 < (int)vec_target.size(); i2++){
        object_tmp = vec_target.at(i2);
        if(object_tmp.type_ == INACTION){//清洗容器得出未激活的目标
            is_target = true;
            final_target = vec_target.at(i2);//当前目标
            buff_angle_ = final_target.angle_;//当前目标的角度
            points_2d = final_target.points_2d_;//当前目标的点
            //big_points_2d = final_target.big_points_2d_;
            target_center = final_target.small_rect_.center;//获取小轮廓的圆心
            //circle(img, big_points_2d[1], 2, Scalar(0,255,255), 2, 8, 0);
            Point2f small_vector = points_2d[0] - points_2d[3];//获取装甲板的高度
            roi_center = final_target.big_rect_.center-BIG_LENTH_R*small_vector;//根据大轮廓的中心加上一定距离得到假定圆心,可修改
            solve_rect = final_target.small_rect_;//获取当前小轮廓的数据
            line(frame, final_target.big_rect_.center, roi_center, Scalar(0,255,255),2);//画出大轮廓到假定圆心的路径
            circle(frame, roi_center, 2, Scalar(0,0,255), 2, 8, 0);//画出假定圆心
            //circle(frame, points_2d[3], 2, Scalar(0,0,255), 2, 8, 0);
        }
    }
    //imshow("src", img);
    return is_target;
}

//待改进,暂时想到比较中心点到roi中心最小的
bool BuffDetector::findCenter_R(Mat &bin_img, Mat &frame){
    bool is_circle = false;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    RotatedRect circle_rect = RotatedRect(Point2f(frame.cols/2,frame.rows/2),Size2f(0,0),0);
    Point2f frame_center(frame.cols/2,frame.rows/2);

    float last_min_distance_target =frame.rows;
    float distance_target = 0;
    double rect_ratio;//比较宽/高
    Point2f circle_r[4];

    vector<RotatedRect>first_screen;

    findContours(bin_img, contours, hierarchy, 0, CHAIN_APPROX_NONE);

    cout<<"轮廓数目："<<contours.size()<<endl;

    for(int j = 0; j < (int)contours.size(); ++j){
        double circle_area = contourArea(contours[j]);

        if(circle_area > 250 || circle_area < 50)//原来为30
            continue;
        RotatedRect temp_circle_rect = fitEllipse(contours[j]);

        if((double)temp_circle_rect.boundingRect().width > (double)temp_circle_rect.boundingRect().height){
            rect_ratio = (double)temp_circle_rect.boundingRect().width/(double)temp_circle_rect.boundingRect().height;
        }
        else {
            rect_ratio = (double)temp_circle_rect.boundingRect().height/(double)temp_circle_rect.boundingRect().width;
        }

//        cout<<"rect_ratio["<<j<<"]="<<rect_ratio<<endl;

        //比例适度要修正.原来为1.1f
        if(rect_ratio > 0.9f && rect_ratio < 1.12f){
            first_screen.push_back(temp_circle_rect);
        }
    }

//    cout<<"符合比例条件的:"<<first_screen.size()<<endl;
    for(int i = 0;i<first_screen.size();++i)
    {
        distance_target = pointDistance(first_screen[i].center,frame_center);
        if(distance_target < last_min_distance_target)
        {
                last_min_distance_target = distance_target;
                circle_rect = first_screen[i];
        }
    }

    circle_rect.points(circle_r);

    circle_center = circle_rect.center;

    is_circle = true;//待确定该标志位的位置
    for(int k = 0;k<4;k++)
    {
        line(frame, circle_r[k],circle_r[(k+1)%4], Scalar(0, 255, 0), 3);
    }
//        drawContours(img, contours, j, Scalar(0,255,0),2);
    ellipse(frame,circle_rect,Scalar(0,255,233),2,8);

    return is_circle;
}

int BuffDetector::buffDetect_Task(Mat &frame){
    imageProcess(frame);
    bool is_target = findTarget(frame);
    int common = 0;
    Mat result_img;
    Mat roi_img;
    Mat roi_power_img;//test
    bin_Img.copyTo(result_img);
    frame.copyTo(roi_img);
    frame.copyTo(roi_power_img);//test

    if(is_target){//可找到未激活目标
        RotatedRect roi_R(roi_center, Size(90,90),0);//画出假定圆心的roi矩形
        Rect roi = roi_R.boundingRect();

        if(roi.tl().x < 0 || roi.tl().y < 0|| roi.br().x > frame.cols || roi.br().y > frame.rows){
            Point TL = roi.tl();
            Point BR = roi.br();
            if(roi.tl().x < 0 || roi.tl().y < 0){
                if(roi.tl().x<0){
                    TL.x = 0;
                }
                else {
                    TL.y = 0;
                }
            }
            else {
                if(roi.br().x>frame.cols){
                    BR.x = frame.cols;
                }
                else {
                    BR.y = frame.rows;
                }
            }
            roi = Rect(TL, BR);
        }

        bin_Img(roi).copyTo(result_img);
        frame(roi).copyTo(roi_img);
        rectangle(frame,roi,Scalar(0,255,200),2,8,0);

        ++find_cnt_;
        if(find_cnt_%2 == 0){//隔帧读数据
            direction_tmp_ = getState();//判断旋转方向
            if(find_cnt_ == 10)
                find_cnt_ = 0;
        }

        if(1)//大神符加速函数
        {
            diff_angle_large = buff_angle_ - last_angle_large;
            cout<<"buff_angle_="<<buff_angle_<<endl;
            cout<<"last_angle="<<last_angle_large<<endl;
            last_angle_large = buff_angle_;

            double timing_point_1 = getTickCount();
            double spt_t = (timing_point_1 - timing_point_2)*1000 / getTickFrequency();
            timing_point_2 = getTickCount();

            current_speed = diff_angle_large/spt_t*CV_PI/180;//变为弧度
            cout<<"diff_angle_large="<<diff_angle_large<<endl;

//            first_frame_time =  asin((current_speed-1.305)/0.785)/1.884;
//            cout<<"first_frame_time="<<first_frame_time<<endl;

        }


        bool is_circle = findCenter_R(result_img, roi_img);

        roi_power_center.x = roi_center.x-45+circle_center.x;//test
        roi_power_center.y = roi_center.y-45+circle_center.y;//test
        RotatedRect roi_power(roi_power_center,Size(400,400),0);//test
        Rect roi_Power = roi_power.boundingRect();//test
        if(roi_Power.tl().x < 0 || roi_Power.tl().y < 0|| roi_Power.br().x > frame.cols || roi_Power.br().y > frame.rows){
            Point TL = roi_Power.tl();
            Point BR = roi_Power.br();
            if(roi_Power.tl().x < 0 || roi_Power.tl().y < 0){
                if(roi_Power.tl().x<0){
                    TL.x = 0;
//                    cout<<"左";
                    if(roi_Power.tl().y<0)
                    {
                        TL.y = 0;
//                        cout<<"上";
                    }
                    if(roi_Power.br().y>frame.rows)
                    {
                        BR.y = frame.rows;
//                        cout<<"下";
                    }
//                    cout<<endl;
                }
                else if(roi_Power.tl().y<0){
                    TL.y = 0;

                    if(roi_Power.br().x>frame.cols)
                    {
                        BR.x = frame.cols;
//                        cout<<"右";
                    }
                    if(roi_Power.tl().x<0)
                    {
                        TL.x = 0;
//                        cout<<"左";
                    }
//                    cout<<"上";
//                    cout<<endl;
                }
            }
            else if(roi_Power.br().x > frame.cols || roi_Power.br().y > frame.rows){
                if(roi_Power.br().x>frame.cols){
                    BR.x = frame.cols;
//                    cout<<"右";
                    if(roi_Power.br().y>frame.rows)
                    {
                        BR.y = frame.rows;
//                        cout<<"下";
                    }
                    if(roi_Power.tl().y<0)
                    {
                        TL.y = 0;
//                        cout<<"上";
                    }
//                    cout<<endl;
                }
                else if(roi_Power.br().y>frame.rows){
                    BR.y = frame.rows;
                    if(roi_Power.tl().x<0)
                    {
                        TL.x = 0;
//                        cout<<"左";
                    }
                    if(roi_Power.br().x>frame.cols)
                    {
                        BR.x = frame.cols;
//                        cout<<"右";
                    }
//                    cout<<"下";
//                    cout<<endl;
                }
            }
            roi_Power = Rect(TL, BR);
        }//test

        frame(roi_Power).copyTo(roi_power_img);//test

        if(is_circle == true){
            double total;
            //小轮廓圆心和R的斜率的反正切，得到一个角
            double theta = atan(double(target_center.y - circle_center.y) / (target_center.x - circle_center.x));
            if(direction_tmp_ != 0){
                total = direction_tmp_*(PRE_ANGLE+theta)*CV_PI/180;//转换为弧度 ！！！此处加上大神符加速函数
//                cout<<"total:"<<total<<endl;
            }
            else {
                total = theta*CV_PI/180;
            }
            double sin_calcu = sin(total);
            double cos_calcu = cos(total);
            Point2f round_center(circle_center.x+roi.tl().x, circle_center.y+roi.tl().y);

            pre_center.x = (target_center.x-round_center.x)*cos_calcu-(target_center.y-round_center.y)*sin_calcu+round_center.x;
            pre_center.y = (target_center.x-round_center.x)*sin_calcu+(target_center.y-round_center.y)*cos_calcu+round_center.y;

            double radio = pointDistance(round_center, pre_center);

            circle(frame, round_center, radio, Scalar(0,255,125),2,8,0);
            circle(frame, pre_center, 3, Scalar(255,0,0),3,8,0);
            line(frame, pre_center, round_center, Scalar(0,255,255),2);
        }
        else{
            Point2f vector_pre = points_2d[0] - points_2d[1];
            if(direction_tmp_ != 0){
                Point2f vector_center = target_center - direction_tmp_ * vector_pre * SMALL_LENTH_R;
                double theta = atan(double(vector_center.y - target_center.y) / (vector_center.x - target_center.x));
                double total = (SMALL_PRE_ANGLE+theta)*CV_PI/180;
                double sin_calcu = sin(total);
                double cos_calcu = cos(total);
                pre_center.x = (vector_center.x-target_center.x)*cos_calcu-(vector_center.y-target_center.y)*sin_calcu+target_center.x;
                pre_center.y = (vector_center.x-target_center.x)*sin_calcu+(vector_center.y-target_center.y)*cos_calcu+target_center.y;

                circle(frame, pre_center, 3, Scalar(0,255,0),3,8,0);
                line(frame, target_center, pre_center, Scalar(255,123,0),2);
                line(frame, target_center, vector_center, Scalar(255,123,0),2);
            }
            else {
                pre_center = target_center;
            }
        }
        solve_buff.run_SolvePnp(frame, solve_rect, buff_angle_);
        angle_x = solve_buff.angle_x;
        angle_y = solve_buff.angle_y;
        dist = solve_buff.dist;

        //cout << "angle_x:" << angle_x << "     angle_y:" << angle_y << "    dist:" << dist <<endl;

    }

    common = auto_control.run(angle_x,angle_y,is_target,diff_angle_);
//    cout <<"current common is:"<<common<<endl;
    //imshow("roi", result_img);
    imshow("roi_img", roi_img);

    imshow("roi_power_img",roi_power_img);//test

    imshow("bin", bin_Img);
    imshow("img", frame);
    return common;
}

int BuffDetector::getState(){
    diff_angle_ = buff_angle_ - last_angle;
    last_angle = buff_angle_;
    if(fabs(diff_angle_)<10 && fabs(diff_angle_)>1e-6){
        d_angle_ = (1 - R) * d_angle_ + R * diff_angle_;
    }
    //cout << "d_angle_:" << d_angle_ << endl;
    if(d_angle_ > 1.5)
        return 1;
    else if(d_angle_ < -1.5)
        return -1;
    else
        return 0;
}

void Object::smallUpdate_Order(){
    points_2d_.clear();
    Point2f points[4];
    small_rect_.points(points);
    Point2f point_up_center = (points[0] + points[1])/2;
    Point2f point_down_center = (points[2] + points[3])/2;
    double up_distance = pointDistance(point_up_center, big_rect_.center);
    double down_distance = pointDistance(point_down_center, big_rect_.center);
    //cout << big_rect_.angle << endl;

    if(up_distance > down_distance){
        angle_ = small_rect_.angle;
        points_2d_.push_back(points[0]);points_2d_.push_back(points[1]);
        points_2d_.push_back(points[2]);points_2d_.push_back(points[3]);
    }
    else {
        angle_ = small_rect_.angle+180;
        points_2d_.push_back(points[2]);points_2d_.push_back(points[3]);
        points_2d_.push_back(points[0]);points_2d_.push_back(points[1]);
    }

    //cout << angle_ << endl;
}

void Object::knowYour_Self(Mat &img){
    Point2f vector_height = points_2d_.at(0) - points_2d_.at(3);

    Point left_center = points_2d_.at(3) - vector_height;
    Point right_center = points_2d_.at(2) - vector_height;

    int width = 5;
    int height = 5;

    Point left1 = Point(left_center.x - width, left_center.y - height);
    Point left2 = Point(left_center.x + width, left_center.y + height);

    Point right1 = Point(right_center.x - width, right_center.y - height);
    Point right2 = Point(right_center.x + width, right_center.y + height);

    Rect left_rect(left1, left2);
    Rect right_rect(right1, right2);

    int left_intensity = getRect_Intensity(img, left_rect);
    int right_intensity = getRect_Intensity(img, right_rect);
    //cout << left_intensity << "  " << right_intensity << endl;
    if(left_intensity <= 20 && right_intensity <= 20){
        type_ = INACTION;
    }
    else {
        type_ = ACTION;
    }
}

double pointDistance(Point2f & p1, Point2f & p2){
    double Dis=pow(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2),0.5);
    return Dis;
}

int getRect_Intensity(const Mat &frame, Rect rect){
    if(rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1
            || rect.width + rect.x > frame.cols || rect.height + rect.y > frame.rows)
        return 255;
    Mat roi = frame(Range(rect.y, rect.y + rect.height), Range(rect.x, rect.x + rect.width) );
    imshow("roi ", roi);
    int average_intensity = static_cast<int>(mean(roi).val[0]);
    return average_intensity;
}


