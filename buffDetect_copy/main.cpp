#include <iostream>
#include <opencv2/opencv.hpp>
#include "detect_buff/buff_detect.h"

using namespace std;
using namespace cv;

int main(){
    VideoCapture cap;
    cap.open("./data/camera_13.avi");
    Mat frame;

    int common = 0;
    int firing_times = 0;

    BuffDetector buff;
    while (true) {
        cap >> frame;
        if(frame.empty())
            break;
        resize(frame, frame, Size(640,480));
#ifdef FPS
        double t1 = (double)getTickCount();//开始计时
#endif

        common = buff.buffDetect_Task(frame);
        
 #ifdef FPS
        double t2 = (double)getTickCount();//开始计时
        double t = (t2 - t1) / getTickFrequency();//结束计时


        cout << "t:" << t << endl;
        int fps = int(1.0 / t);//转换为帧率
        cout << "fps:" << fps << endl;
        Point Fps = Point(10,20);
        char text[50];
        sprintf(text,"FPS:%d",fps);
        putText(frame,text,Fps,FONT_HERSHEY_PLAIN,1,Scalar(100,200,0),1,8,false);
        cout<<"-----------------------------"<<endl;

#endif
#ifdef auto_control_text
        //开火 5 1 3
        //跟随 3 1
        //不跟随 2 0
        //复位 5 6

        char times[50];
        sprintf(times,"Firing times:%d",firing_times);
        Point F_times = Point(frame.cols-200,20);
        putText(frame,times,F_times,FONT_HERSHEY_PLAIN,1,Scalar(100,200,100),1,8,false);

        Point autocommon = Point(frame.cols-100,frame.rows-20);
        if(common == 1){
            putText(frame,"FIRE",autocommon,FONT_HERSHEY_PLAIN,1,Scalar(100,200,100),1,8,false);
            firing_times ++;
        }else if(common == 2){
            putText(frame,"RESET",autocommon,FONT_HERSHEY_PLAIN,1,Scalar(100,200,100),1,8,false);
        }else if(common == 3){
            putText(frame,"FOLLOW",autocommon,FONT_HERSHEY_PLAIN,1,Scalar(100,200,100),1,8,false);
        }else if(common == 4){
            putText(frame,"UNFOLLOW",autocommon,FONT_HERSHEY_PLAIN,1,Scalar(100,200,100),1,8,false);
        }
#endif
        imshow("img", frame);
        char c = waitKey(0);//20
        if(c == 27)
            break;

//        int key =waitKey(0);
//        if((char)key== 32)
//            continue;
    }
}
