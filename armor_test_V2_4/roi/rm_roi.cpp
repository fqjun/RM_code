#include "rm_roi.h"

ArmorROI setImg(Mat src,bool is_Lost_target,short int cnt,RotatedRect last_roi_rect)
{
    ArmorROI Rrect;
    Mat set_img;
    Point tl;
    int W;
    int H;
    if(is_Lost_target == false){
        if(cnt <= LOSE_CNT_MAX*0.5){
            W = last_roi_rect.boundingRect().width * 4;//1.5
            H = last_roi_rect.boundingRect().height *4;//2.2
        } else if (LOSE_CNT_MAX*0.5 < cnt &&cnt <= LOSE_CNT_MAX){
            W = last_roi_rect.boundingRect().width * 5;//2.5
            H = last_roi_rect.boundingRect().height * 6;//3.2
        } else{
            W = last_roi_rect.boundingRect().width;
            H = last_roi_rect.boundingRect().height;
        }

        /*--- ROI区域安全处理 ---*/
        tl = Point(last_roi_rect.center.x - W*0.5,last_roi_rect.center.y - H*0.5);
        if (tl.x < 0){
            tl.x = 0;//如果左上角点x超出尺寸，x =  0
        }
        if (tl.y < 0){
            tl.y = 0;//如果左上角点y超出尺寸，y = 0
        }
        if (last_roi_rect.center.x - W*0.5 < 0){
            W = W + (last_roi_rect.center.x - W*0.5 );
            // 如果宽度超出了左边界，设置为到边界的值
        }
        if (tl.x + W > src.cols){
            W = W - (tl.x + W - src.cols);
            // 如果宽度超出了右边界，设置为到边界的值
        }
        if (last_roi_rect.center.y - H*0.5 < 0){
            H = H + (last_roi_rect.center.y - H*0.5 );
            // 如果高度超出了上边界，设置为到边界的值
        }
        if (tl.y + H > src.rows){
            H = H - (tl.y + H - src.rows );
            // 如果高度超出了下边界，设置为到边界的值
        }
        /*--- ROI区域安全处理 ---*/

        Rect roi = Rect(tl.x,tl.y,W,H);
        src(roi).copyTo(set_img);
        Rrect.getParam(last_roi_rect,tl,W,H,set_img);
    }else{
        src.copyTo(set_img);
        Rrect.getParam(last_roi_rect,tl,0,0,set_img);
        Rrect.resetParam();
    }
    return Rrect;
}

ArmorROI setNumber(Mat src,bool is_Lost_target,short int cnt,RotatedRect last_roi_rect)
{
    ArmorROI Rrect;
    Mat set_img;
    Point tl;
    int W;
    int H;
    if(is_Lost_target == false){
        if(cnt <= LOSE_CNT_MAX*0.5){
            W = last_roi_rect.boundingRect().width ;//1.5
            H = last_roi_rect.boundingRect().height ;//2.2
        } else if (LOSE_CNT_MAX*0.5 < cnt &&cnt <= LOSE_CNT_MAX){
            W = last_roi_rect.boundingRect().width * 0.8;//2.5
            H = last_roi_rect.boundingRect().height * 0.8;//3.2
        } else{
            W = last_roi_rect.boundingRect().width;
            H = last_roi_rect.boundingRect().height;
        }

        /*--- ROI区域安全处理 ---*/
        tl = Point(last_roi_rect.center.x - W*0.5,last_roi_rect.center.y - H*0.5);
        if (tl.x < 0){
            tl.x = 0;//如果左上角点x超出尺寸，x =  0
        }
        if (tl.y < 0){
            tl.y = 0;//如果左上角点y超出尺寸，y = 0
        }
        if (last_roi_rect.center.x - W*0.5 < 0){
            W = W + (last_roi_rect.center.x - W*0.5 );
            // 如果宽度超出了左边界，设置为到边界的值
        }
        if (tl.x + W > src.cols){
            W = W - (tl.x + W - src.cols);
            // 如果宽度超出了右边界，设置为到边界的值
        }
        if (last_roi_rect.center.y - H*0.5 < 0){
            H = H + (last_roi_rect.center.y - H*0.5 );
            // 如果高度超出了上边界，设置为到边界的值
        }
        if (tl.y + H > src.rows){
            H = H - (tl.y + H - src.rows );
            // 如果高度超出了下边界，设置为到边界的值
        }
        /*--- ROI区域安全处理 ---*/

        Rect roi = Rect(tl.x,tl.y,W,H);
        src(roi).copyTo(set_img);
        Rrect.getParam(last_roi_rect,tl,W,H,set_img);
    }else{
        src.copyTo(set_img);
        Rrect.getParam(last_roi_rect,tl,0,0,set_img);
        Rrect.resetParam();
    }
    return Rrect;
}
