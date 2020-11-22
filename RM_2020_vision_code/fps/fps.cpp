#include "fps.h"

Fps::Fps(){
    time = 0.0;
    fps = 0;
    time_account = 0;
    fps_account = 0;
    fps_average = 0.0;
}

Fps::~Fps(){
}

void Fps::starttheTime(){

    time = double(getTickCount());
    time_account += 1;

}

void Fps::endtheTime(){

    time = (double(getTickCount() - time)) / getTickFrequency();
    fps = int(1.0/time);
    

}
void Fps::displayframeRate(){

    cout<<"FPS: "<<fps<<endl;

}
void Fps::computetheAverage(){

    fps_account += fps;
    fps_average = fps_account / time_account;
    cout<<"FPS_average: "<< fps_average<<endl;

}
