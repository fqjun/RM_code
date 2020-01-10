#ifndef RM_LINK_H
#define RM_LINK_H

#include "configure.h"
#include "rm_videocapture.h"
#include "rm_armorfitted_1.h"
#include "serialport.h"
#include "kalmantest.h"

extern double g_runtime;

struct Control_Information{
    int my_color;
    int now_run_mode;
    int serial_plan;
    int armor_size;
    int my_Robot_ID;
};

class RM_Vision_Init
{
public:

    RM_Vision_Init();
    ~RM_Vision_Init();

    void Run();
    bool is_exit();
    bool is_continue();
    void updateControl_information(int arr[REC_BUFF_LENGTH]);//更新控制信息

    Control_Information g_Ctrl;
    /** Camera Srart **/

    VideoCapture capture;
    #ifdef CAMERA
    RM_VideoCapture cap;
    #endif
    /** Camera Srart **/

    #if SERIAL_IS_OPEN ==1
    SerialPort serial;
    #endif

private:

    /** param initial **/
    Mat src_img;
    /** param initial **/

    /** function initial **/
    RM_ArmorFitted armor;//装甲板识别
//    RM_BigChrysanthemum agency;//大风车识别
    /** function initial **/



};

#endif // RM_LINK_H
