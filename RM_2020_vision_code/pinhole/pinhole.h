#ifndef PINHOLE_H
#define PINHOLE_H

#include "configure.h"
#include "control/debug_control.h"

class Pinhole
{
private:
    
public:
    Pinhole(){};
    ~Pinhole(){};

    float focal_length = 0;
    float distance = 0;


    float getDistance(float &pix,float &focal_length,float &target);
    float getfocalLength(float &pix,float &distance,float &target);


};



#endif // !PINHOLE_H
