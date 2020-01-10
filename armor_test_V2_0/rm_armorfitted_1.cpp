#include "rm_link.h"

void RM_ArmorFitted::imageProcessing(Mat frame,int my_color)
{
    src_img =frame;
    copy_img=src_img.clone();
    dst_img = src_img;
  /*-----------------------------------------ROI----------------------------------------------------*/

    if(is_last_data_catch == true && is_Lost_target == false){
        roi = setImg(src_img,is_Lost_target,lose_target_cnt,last_armor);
        roi_img = roi.ROI_img;
    }else if(is_Lost_target == false && is_last_data_catch == false && lose_target_cnt <= LOSE_CNT_MAX){
        roi = setImg(src_img,is_Lost_target,lose_target_cnt,last_armor);
        roi_img = roi.ROI_img;
        lose_target_cnt += 1;
    }else {
        lose_target_cnt = 0;
        is_Lost_target = true;
        roi = setImg(src_img,is_Lost_target,lose_target_cnt,last_armor);
        roi_img = roi.ROI_img;
    }
    is_last_data_catch = false;

//----------------------------------预处理-----------------------------------------
    cvtColor(roi_img,lab_img,COLOR_RGB2Lab);
    cvtColor(roi_img,gray_img,COLOR_RGB2GRAY);
#ifdef NUMBER
    Mat bin_img;
    Mat roi_number_img;
//    medianBlur(gray_img,gray_img,3);
//    GaussianBlur(gray_img,gray_img,Size(3,3),0,0);

//    adaptiveThreshold( gray_img, bin_img);


    Point2f point1;
    Point2f point_center;
//    point1.x=bin_img.cols/2;
//    point1.y=bin_img.rows/2;
//    if(last_armor.center.x <= roi_img.cols/2)
//    {
//        point1.x=bin_img.cols/2;
//    }
//    else
//    {
//        point_center.x=last_armor.center.x-(last_armor.center.x-roi_img.cols/2);
//        point1.x=bin_img.cols/2+(bin_img.cols/2-point_center.x);
//    }

//    if(last_armor.center.y <= roi_img.rows/2)
//    {
//        point1.y=bin_img.rows/2;
//    }
//    else
//    {
//        point_center.y=last_armor.center.y-(last_armor.center.y-roi_img.rows/2);
//        point1.y=bin_img.rows/2+(bin_img.rows/2-point_center.y);
//    }
    Size armor_number;
        armor_number.width=last_armor.size.width*0.5;
        armor_number.height=last_armor.size.height*1.6;
    RotatedRect number=RotatedRect(point1,armor_number,last_armor.angle);
//    roi_number=setNumber(bin_img,is_Lost_target,lose_target_cnt,number);
    roi_number_img=roi_number.ROI_img;


//    imshow("roi_number_img",roi_number_img);

    Mat element = getStructuringElement(MORPH_RECT, Size(9,9));

//    GaussianBlur(bin_img,bin_img,Size(3,3),0,0);
//    morphologyEx(bin_img,bin_img,MORPH_CLOSE,element);//
//    morphologyEx(bin_img,bin_img,MORPH_BLACKHAT,element);//
//    medianBlur(bin_img,bin_img,3);
//        dilate(bin_img,bin_img,element);
//    morphologyEx(bin_img,bin_img,MORPH_CLOSE,element);//闭运算
//    morphologyEx(bin_img,bin_img,MORPH_OPEN,element);//闭运算



    medianBlur(bin_img,bin_img,3);
    dilate(bin_img,bin_img,element);
    imshow("bin_img",bin_img);
#endif

        if(my_color == BLUE)
        {
#ifdef TRACKBAR
            namedWindow("window");

            createTrackbar("Lmin","window",&g_RLmin,g_max,nullptr);
            createTrackbar("Lmax","window",&g_RLmax,g_max,nullptr);
            createTrackbar("Amin","window",&g_RAmin,g_max,nullptr);
            createTrackbar("Amax","window",&g_RAmax,g_max,nullptr);
            createTrackbar("Bmin","window",&g_RBmin,g_max,nullptr);
            createTrackbar("Bmax","window",&g_RBmax,g_max,nullptr);
#endif
            inRange(lab_img,Scalar(g_RLmin,g_RAmin,g_RBmin),Scalar(g_RLmax,g_RAmax,g_RBmax),mask);//红 (0,141,20) (130,205,101)
        }
        else if(my_color == RED)
        {
#ifdef TRACKBAR
            namedWindow("window");

            createTrackbar("Lmin","window",&g_BLmin,g_max,nullptr);
            createTrackbar("Lmax","window",&g_BLmax,g_max,nullptr);
            createTrackbar("Amin","window",&g_BAmin,g_max,nullptr);
            createTrackbar("Amax","window",&g_BAmax,g_max,nullptr);
            createTrackbar("Bmin","window",&g_BBmin,g_max,nullptr);
            createTrackbar("Bmax","window",&g_BBmax,g_max,nullptr);
#endif
            inRange(lab_img,Scalar (g_BLmin,g_BAmin,g_BBmin), Scalar(g_BLmax,g_BAmax,g_BBmax),mask);//蓝 (100,80,110) (200,190,210) 180   (18,150,140),Scalar(190,205,206)  (60, 20, 120), Scalar(140, 250, 255)
            //(60, 20, 120), Scalar(140, 250, 255)
        }

}

void RM_ArmorFitted::armorFitted()
{
    is_last_data_catch = false;

    imshow("mask",mask);

    float delta_rc_angle=0;
    float delta_lc_angle=0;

    vector < vector < Point > > contours;
    vector < RotatedRect > light_rect;//最小外接旋转,即灯条
//--------------------------------查找灯条-------------------------------------------------
    findContours(mask, contours,  RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());
    for (int i = 0; i < (int) contours.size(); ++i)
    {
        if (contours.size() <= 1)
        {
            cout<<"boom"<<endl;
            break;
        }
        RotatedRect temp_rect= minAreaRect(contours[i]);//最小外接旋转矩形

        double max_rrect_len = MAX(temp_rect.size.width, temp_rect.size.height);
        double min_rrect_len = MIN(temp_rect.size.width, temp_rect.size.height);

//            bool if1 = ((fabs(temp_rect.angle) >= 135.0 )&&(fabs(temp_rect.angle) <= 180.0 ) && max_rrect_len > min_rrect_len); // 往左倾斜限制 待改进
//            bool if2 = ((fabs(temp_rect.angle) >= 0.0 )&&(fabs(temp_rect.angle) <= 45.0 ) && max_rrect_len > min_rrect_len);// 往右倾斜限制 待改进
//            bool if4 = ((min_rrect_len / max_rrect_len <0.50)&&(min_rrect_len / max_rrect_len>0.10))  ; // 灯条的长宽比

        bool if1 = (fabs(temp_rect.angle) <=45 && temp_rect.size.height>temp_rect.size.width);// 往左倾斜限制 待改进
        bool if2 = (fabs(temp_rect.angle) >45 && temp_rect.size.width>temp_rect.size.height);// 往右倾斜限制 待改进
        bool if3 = max_rrect_len > min_light_height; // 灯条的最小高度
        bool if4 = (max_rrect_len/min_rrect_len >= 2.1)  ; // 灯条的长宽比

//            if(min_rrect_len!=0 )
//            cout<<"比值："<<max_rrect_len / min_rrect_len<<endl;

        bool if5=(max_rrect_len / min_rrect_len < 15);

        if ((if1 || if2) && if3 && if4&&if5)//判断是否为灯条的形状
        {
            light_rect.push_back(temp_rect);

#ifdef DEDUG
            Point2f light_point[4];

            temp_rect.points(light_point);
            for(size_t i=0;i<4;i++)
            {
               line(copy_img,light_point[i] + (Point2f)roi.tl,light_point[(i+1)%4] + (Point2f)roi.tl,Scalar(0,255,255),1,4,0);//黄色
            }
#endif
        }
    }

#ifdef DEDUG_1
//--------------------------------------------双循环两两匹配灯条---------------------------------------

    Point2f _pt[4],lu_i,ru_i,lu_j,ru_j;

    float light_addH_max = 0;

    for(size_t i=0; i<light_rect.size(); ++i)
    {

        RotatedRect rect_i = light_rect[i];

        Point center_i = rect_i.center;
        float xi =center_i.x;
        float yi =center_i.y;
        float leni = MAX(rect_i.size.width, rect_i.size.height);
        float width_i=MIN(rect_i.size.width,rect_i.size.height);
//            float anglei = fabs(rect_i.angle);
        float anglei;
        float area_i=rect_i.size.area();

        rect_i.points(_pt);
        if(anglei>45.0)
        {
            lu_i = _pt[2];
            ru_i = _pt[3];
        }
        else
        {
            lu_i = _pt[1];
            ru_i = _pt[2];
        }

        anglei=(atan((lu_i.y-ru_i.y)/(lu_i.x-ru_i.x))) * 180 / CV_PI;

        for (size_t j = i + 1; j < light_rect.size(); j++)
        {

            RotatedRect rect_j = light_rect[j];
            Point center_j = rect_j.center;
            float xj = center_j.x;
            float yj = center_j.y;
            float lenj = MAX(rect_j.size.width, rect_j.size.height);
            float width_j=MIN(rect_j.size.width,rect_j.size.height);

            float area_j=rect_j.size.area();
//                float anglej=  fabs(rect_j.angle);
            float anglej;
            rect_j.points(_pt);
            if(anglej > 45.0)
            {
                lu_j = _pt[2];
                ru_j = _pt[3];
            }
            else
            {
                lu_j = _pt[1];
                ru_j = _pt[2];
            }


            anglej=(atan((lu_j.y-ru_j.y)/(lu_j.x-ru_j.x))) * 180 / CV_PI;

            if(anglei<-45)
            {
                anglei=anglei+90;
            }

            if(anglej<-45)
            {
                anglej=anglej+90;
            }

            float delta_w =fabs(xj - xi);
            float delta_h = fabs(yj-yi);
//                float len_max =MAX(lenj,leni);
//            float lr_rate = leni > lenj ? leni / lenj : lenj / leni;
            float area_max=MAX(area_i,area_j);
            float area_min=MIN(area_i,area_j);
            float area_ratio=area_max/area_min;

            float angle_left,angle_right;
            float width_left,width_right;
            float light_distance = POINT_DIST(center_i,center_j);
            float delta_center_y=fabs(rect_j.center.y-rect_i.center.y);

            //左右灯条斜率
            if(rect_i.center.x>rect_j.center.x)
            {
                angle_left=anglej;
                angle_right=anglei;
                width_left=width_j;
                width_right=width_i;
            }
            else
            {
                angle_left=anglei;
                angle_right=anglej;
                width_left=width_i;
                width_right=width_j;
            }

#ifdef LR_rate
            if(lr_rate<2&&leni>0&&lenj>0)
            {
                a++;
                lr_rate_max=MAX(lr_rate_max,lr_rate);
                box+=lr_rate;
                average=box/a;

            }

            cout<<"左棱长比值："<<lr_rate<<endl;
            cout<<"最大值："<<lr_rate_max<<endl;
            write<<average<<endl;

#endif
#ifdef Len
       if(1)
       {
           a++;
           cout<<"leni:"<<leni<<"   "<<"lenj:"<<lenj<<endl;
           cout<<"abs:"<<abs(yi - yj) <<endl;
           box+=abs(yi - yj);
           average=box/a;
           write<<average<<endl;
       }
#endif

//            bool condition1 = delta_w > min_light_delta_w && delta_w < max_light_delta_w;//状况一：判断两灯条的左右距离差差
            bool  condition2= (delta_center_y < (rect_j.center.y+rect_i.center.y)*0.50 );//灯条的高度差 is_height_diff_catch
//                    cout<<"delta_center_y:"<<delta_center_y<<endl;
//                    cout<<"对比："<<(rect_j.center.y+rect_i.center.y)*0.5<<endl;
            bool condition3=(light_distance < MAX (leni,lenj) * 5);//灯条间的距离 (待改进)  light_distance_catch
//                    cout<<"light_distance:"<<light_distance<<endl;
//                    cout<<"对比："<<MAX (leni,lenj) * 5<<endl;
//                    cout<<"i的高："<<leni<<"   j的高："<<lenj<<endl;

            //判断目标左右灯柱的比例值(可选)
            bool condition4= delta_h < (rect_j.center.y+rect_i.center.y)*0.5;
            bool condition5=area_ratio<2;//灯条面积比值 (待改进)
            cout<<"condition4:"<<condition4<<endl;
            if(/*condition1*/ /*&&*/ condition2 && condition3 && condition4 && condition5)
            {
                RotatedRect candidate_rect = boundingRRect(rect_i, rect_j);//两灯条矩形的拟合矩形

#ifdef DEDUG

//                    float delta_lc_angle_max=0,delta_rc_angle_max=0;

                float obj_rect_angle=candidate_rect.angle;//拟合矩形的角度

                delta_lc_angle=fabs(obj_rect_angle-angle_left);
                delta_rc_angle=fabs(obj_rect_angle-angle_right);

                bool delta_light_center_angle_max = delta_lc_angle < 9 && delta_rc_angle < 9;//左、右灯条与拟合矩形的角度之差
//                    cout<<"左差值："<<delta_lc_angle<<"   "<<"右差值："<<delta_rc_angle<<endl;

                bool center_rect_angle = fabs(obj_rect_angle)<40;//拟合矩形的合适斜率
//                    cout<<"obj_rect_angle:"<<obj_rect_angle<<endl;





                bool is_light_angle_catch;//右正左负
                if(angle_left /angle_right > 0){
                    //同侧
//                        cout<<fabs(angle_left-angle_right)<<endl;
                    is_light_angle_catch = (fabs(angle_left-angle_right) <= 10);
                } else if(angle_left < 0 && angle_right > 0){
                    //--- \ / --- 内八
//                        cout<<"内八："<<fabs(angle_left-angle_right)<<endl;
                    is_light_angle_catch = (0 <= fabs(angle_left-angle_right) && fabs(angle_left-angle_right) <= 10);
                } else if(angle_left > 0 && angle_right < 9){
                    //--- / \ --- 外八
//                        cout<<"外八："<<fabs(angle_left-angle_right)<<endl;
                    is_light_angle_catch = (0 <= fabs(angle_left-angle_right) && fabs(angle_left-angle_right) <= 10);
                } else if((angle_left ==0 || angle_left == -0 )&& angle_right != 0){
                    //左边竖直
//                        cout<<"左边竖直："<<fabs(angle_left-angle_right)<<endl;
                    is_light_angle_catch = /*false*/(/*170 <= fabs(angle_left-angle_right) || */fabs(angle_left-angle_right) <= 10);
                } else if(angle_left !=0 && (angle_right == 0 || angle_right == -0)){
                    //右边竖直
                    is_light_angle_catch = /*false*/(/*170 <= fabs(angle_left-angle_right) || */fabs(angle_left-angle_right) <= 10);
                }



//                    cout<<obj_rect_angle<<endl;

                if( delta_light_center_angle_max
                        && center_rect_angle
                        && is_light_angle_catch)
                {
                    is_last_data_catch = true;//检测到装甲板，则下一帧会标识上一帧有数据
                    is_Lost_target = false;//未丢失目标
                    RotatedRect last_armor_rect = boundingRRect(rect_i,rect_j);
//                    RotatedRect last_rect = boundingRRect_roi(rect_i,rect_j,roi);
//                                cout<<"rect_i.size.height="<<rect_i.size.height<<endl;
//                                cout<<"rect_j.size.height="<<rect_j.size.height<<endl;
                    if( leni + lenj >=light_addH_max)
                    {
                        light_addH_max = leni + lenj;
                        real_rect.rect = last_armor_rect;
                        Point2f vtx[4];
                        last_armor_rect.points(vtx);
                        for(int j=0;j<4;j++)
                        {
                            line(copy_img,vtx[j], vtx[(j + 1) % 4],Scalar(255,105,180),2,8,0);
                        }
                        circle(copy_img,last_armor_rect.center,2,Scalar(255,105,180),2,8,0);

//                        is_Lost_target = false;
//                        lose_target_cnt = 0;
//                        last_armor = last_rect;
//                        Point2f vty[4];
//                        last_rect.points(vty);
//                        for(int j=0;j<4;j++)
//                        {
//                            line(dst_img,vty[j], vty[(j + 1) % 4],Scalar(255,105,180),2,8,0);
//                        }
//                        circle(dst_img,last_rect.center,2,Scalar(255,105,180),2,8,0);
                    }

//                                cout<<"left:"<<angle_left<<"   "<<"right:"<<angle_right<<endl;
//                                 cout<<"左宽："<<width_left<<"    右宽："<<width_right<<endl;
//                                 cout<<"宽比值："<<MAX(width_left,width_right)/MIN(width_left,width_right)<<endl;
                        Point2f vertice[4];
                        candidate_rect.points(vertice);
                        for (int i = 0; i < 4; i++)
                            line(dst_img, vertice[i] + (Point2f)roi.tl, vertice[(i + 1) % 4] + (Point2f)roi.tl, Scalar(255, 255, 255), 2);
                }
#endif
//                cout<<"------------------------------------"<<endl;
//                    cout<<"左差值："<<delta_lc_angle_max<<"   "<<"右差值："<<delta_rc_angle_max<<endl;
            }
        }
    }
//    cout<<"light_addH_max="<<light_addH_max<<endl;
//    cout<<"armor_light_pair.size="<<armor_light_pair.size()<<endl;
//第二次循环结束
#endif

    #if IS_KF_PREDICT_ARMOR_OPEN ==1
     Point2f kalman_point = kalman.point_Predict(g_runtime,real_rect.rect.center);
     real_rect.rect.center.x=kalman_point.x;
     real_rect.rect.center.y=kalman_point.y;

     circle(copy_img,kalman_point,2,Scalar(255,105,255),2,8,0);
    #endif
    #if SERIAL_IS_OPEN == 1
     SerialPort::sendData(model,model_select,real_rect.rect.center.x,real_rect.rect.center.y,depth);
    #endif

    #if SHOW_OUTPUT_IMG==1
    imshow("dst_img",dst_img);
//    imshow("src_img",src_img);
    imshow("copy_img",copy_img);
//    imshow("roi_img",roi_img);
    #endif
}

RotatedRect RM_ArmorFitted::boundingRRect_roi( cv::RotatedRect  left,  cv::RotatedRect  right,ArmorROI &roirect)//拟合矩形函数
{

    Point2f  pl = left.center, pr = right.center;
    Point2f center = (pl + pr) / 2.0;
    center.x = center.x + roirect.tl.x;
    center.y = center.y + roirect.tl.y;

//    cv::Size2f wh_l = left.size;
//    cv::Size2f wh_r = right.size;
    // 这里的目标矩形的height是之前灯柱的width
    double width_l = MIN(left.size.width, left.size.height);
    double width_r = MIN(right.size.width, right.size.height);
    double height_l = MAX(left.size.width, left.size.height);
    double height_r = MAX(right.size.width, right.size.height);

    float width = POINT_DIST(pl, pr) - (width_l + width_r) /*/ 2.0*/;
    float height = max(height_l, height_r);
    //float height = (wh_l.height + wh_r.height) / 2.0;
//    float angle = atan2(right.center.y - left.center.y, right.center.x - left.center.x);//jiao du fanwei zai 0~180
    float angle = atan((right.center.y-left.center.y)/(right.center.x-left.center.x));//-90~0~90

//    cout<<(atan((right.center.y-left.center.y)/(right.center.x-left.center.x)))* 180 / CV_PI<<endl;

     RotatedRect Rrect = RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
     return Rrect;
}

RotatedRect RM_ArmorFitted::boundingRRect( cv::RotatedRect  left,  cv::RotatedRect  right)//拟合矩形函数
{

    Point2f  pl = left.center, pr = right.center;
    Point2f center = (pl + pr) / 2.0;


//    cv::Size2f wh_l = left.size;
//    cv::Size2f wh_r = right.size;
    // 这里的目标矩形的height是之前灯柱的width
    double width_l = MIN(left.size.width, left.size.height);
    double width_r = MIN(right.size.width, right.size.height);
    double height_l = MAX(left.size.width, left.size.height);
    double height_r = MAX(right.size.width, right.size.height);

    float width = POINT_DIST(pl, pr) - (width_l + width_r) / 2.0;
    float height = max(height_l, height_r);
    //float height = (wh_l.height + wh_r.height) / 2.0;
//    float angle = atan2(right.center.y - left.center.y, right.center.x - left.center.x);//jiao du fanwei zai 0~180
    float angle = atan((right.center.y-left.center.y)/(right.center.x-left.center.x));//-90~0~90

//    cout<<(atan((right.center.y-left.center.y)/(right.center.x-left.center.x)))* 180 / CV_PI<<endl;

    return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
}

#ifdef NUMBER
void RM_ArmorFitted::adaptiveThreshold( const cv::Mat& inImg, cv::Mat& outImg)
{
    if (outImg.empty())
    {
        outImg.create(inImg.size(), inImg.depth());
    }
    int S = inImg.cols/8 ;//可修改  8  24~32
    int T = 15;//可修改  15
    int count=0;

//    int index;
    int row, col, x1, y1, x2, y2;
    long double brs, bls, trs, tls, value;
    int s2 = S/2;
    Mat sum(inImg.rows+1, inImg.cols+1, CV_32FC1);
    integral(inImg, sum);//创建积分图像
    for (row = 0; row < inImg.rows; row++)
    {
        const uchar* ptr = inImg.ptr<uchar>(row);
        uchar* ptr_out = outImg.ptr<uchar>(row);
        for (col = 0; col < inImg.cols; col++)
        {
            // set the SxS region
            x1 = col-s2; x2 = col+s2;
            y1 = row-s2; y2 = row+s2;
            // check the border
            if (x1 < 0) x1 = 0;
            if (x2 >= inImg.cols) x2 = inImg.cols-1;
            if (y1 < 0) y1 = 0;
            if (y2 >= inImg.rows) y2 = inImg.rows-1;
            count = (x2-x1)*(y2-y1);
            brs = sum.at<int>(y2, x2);
            bls = sum.at<int>(y2, x1);
            trs = sum.at<int>(y1, x2);
            tls = sum.at<int>(y1, x1);
            value = brs+tls-trs-bls;
            if ((ptr[col]*count) < (long)(value*(100-T)/100))
            {
                ptr_out[col] = 255;
            }
            else
            {
                ptr_out[col] = 0;
            }
        }
    }
}
#endif
