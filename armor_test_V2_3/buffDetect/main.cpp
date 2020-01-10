#include <iostream>
#include <opencv2/opencv.hpp>
#include "/home/hzh/QT_work/buff_detect/detect_buff/buff_detect.h"
using namespace std;
using namespace cv;

int main(){
    VideoCapture cap;
    cap.open("../buff_detect/data/camera_13.avi");
    Mat frame;

    BuffDetector buff;
    while (true) {
        cap >> frame;
        if(frame.empty())
            break;
        resize(frame, frame, Size(640,480));
        double t = getTickCount();
        buff.buffDetect_Task(frame);
        t = ((double)getTickCount() - t) / getTickFrequency();
        //cout << "t:" << t << endl;
//        double fps = 1.0 / t;
//        cout << "fps:" << fps << endl;

        char c = waitKey(1);
        if(c == 27)
            break;
    }
}
