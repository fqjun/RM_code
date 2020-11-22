#ifndef FPS_H
#define FPS_H

#include "configure.h"
#include "debug_control.h"

class Fps
{
public:
    Fps();
    ~Fps();

    void starttheTime();
    void endtheTime();
    void computetheAverage();
    void displayframeRate();

    double time;

private:
    //Timing
    int fps;

    //Average
    int time_account;
    int fps_account;
    float fps_average;


};

#endif // FPS_H
