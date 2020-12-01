#include "pinhole.h"

/**
 * @brief 获取距离
 * 
 * @param pix 像素长度
 * @param focal_length 焦距 mm 
 * @param target 目标长度 mm
 * @return float 
 */
float Pinhole::getDistance(float *pix,float focal_length,float target){

    distance = *pix * focal_length / target;
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
float Pinhole::getfocalLength(float *pix,float distance,float target){

    focal_length = *pix * distance / target; 
    return focal_length;

}