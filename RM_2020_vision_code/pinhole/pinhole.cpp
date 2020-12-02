#include "pinhole.h"

/**
 * @brief 获取距离
 * 
 * @param pix 像素长度
 * @param focal_length 焦距 mm 
 * @param target 目标长度 mm
 * @return float 
 */
float Pinhole::getDistance(float &pix,float &focal_length,float &target){

    distance = target * focal_length / pix;
    cout<<"distance: "<<distance<<endl;
    return distance;

}

/**
 * @brief 获取焦距
 * 
 * @param pix 像素长度
 * @param distance 实际距离 mm
 * @param target 目标长度 mm
 * @return float 
 */
float Pinhole::getfocalLength(float &pix,float &distance,float &target){

    focal_length = pix * distance / target; 
    cout<<"focal_length: "<<focal_length<<endl;
    return focal_length;

}