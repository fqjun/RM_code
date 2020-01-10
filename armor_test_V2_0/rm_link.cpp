#include "rm_link.h"

double g_runtime;

#ifdef CAMERA
RM_Vision_Init::RM_Vision_Init():capture(CAPTURE_DEFULT),cap(ISOPEN_INDUSTRY_CAPTURE){
}

RM_Vision_Init::~RM_Vision_Init(){
    cap.~RM_VideoCapture();
}
#endif
#ifdef VEDIO
RM_Vision_Init::RM_Vision_Init(){capture.open(CAPTURE_DEFULT);}
RM_Vision_Init::~RM_Vision_Init(){}
#endif

void RM_Vision_Init::updateControl_information(int arr[REC_BUFF_LENGTH]){
    /*更新模式信息 update mode*/
    switch (arr[1]) {
    case SUP_SHOOT:
        g_Ctrl.now_run_mode = SUP_SHOOT;
        break;
    case ENERGY_AGENCY:
        g_Ctrl.now_run_mode = ENERGY_AGENCY;
        break;
    case SENTRY_MODE:
        g_Ctrl.now_run_mode = SENTRY;
        break;
    case BASE_MODE:
        g_Ctrl.now_run_mode = BASE_MODE;
        break;
    default:
        g_Ctrl.now_run_mode = SUP_SHOOT;
        break;
    }

    /*更新当前机器人ID update Robot ID*/
    switch (arr[2]) {
    case HERO:
        g_Ctrl.my_Robot_ID = HERO;
        break;
    case ENGINEERING:
        g_Ctrl.my_Robot_ID = ENGINEERING;
        break;
    case INFANTRY:
        g_Ctrl.my_Robot_ID = INFANTRY;
        break;
    case UAV:
        g_Ctrl.my_Robot_ID = UAV;
        break;
    case SENTRY:
        g_Ctrl.my_Robot_ID = SENTRY;
        break;
    default:
        g_Ctrl.my_Robot_ID = INFANTRY;
        break;
    }

    /*更新颜色信息　update color*/
    switch (arr[3]) {
    case BLUE:
        g_Ctrl.my_color = BLUE;
        break;
    case RED:
        g_Ctrl.my_color = RED;
        break;
    default:
        g_Ctrl.my_color = ALL_COLOR;
        break;
    }


    /*更新陀螺仪数据 update gyroscope data*/
}


void RM_Vision_Init::Run()
{
#ifdef CAMERA
    if(cap.isindustryimgInput())
    {
        src_img = cvarrToMat(cap.iplImage,true);//这里只是进行指针转换，将IplImage转换成Mat类型
    }
    else
    {
        capture >> src_img;
    }
#endif

#ifdef VEDIO
    capture >> src_img;
#endif

#if SERIAL_IS_OPEN ==1
    int ctrl_arr[REC_BUFF_LENGTH];
    SerialPort::receiveData(ctrl_arr);
    updateControl_information(ctrl_arr);
#endif

#if MY_COLOR == 1
    g_Ctrl.my_color = RED;
#elif MY_COLOR == 2
    g_Ctrl.my_color = BLUE;
#elif MY_COLOR == 3
    cout<<"串口控制"<<endl;
#endif

//    resize(src_img,src_img,Size(640,512));
#if COUT_STATE == 1
    cout<<"mode: "<<g_Ctrl.now_run_mode<<endl;
#endif
    /** Change Mode　**/
    switch (g_Ctrl.now_run_mode)
    {
    /**-Support Shooting mode-**/
    case SUP_SHOOT:
        {
            imshow("src_img",src_img);
            armor.imageProcessing(src_img,g_Ctrl.my_color);
            armor.armorFitted();
        }
        break;
    /** Energy Agency Mode **/
    case ENERGY_AGENCY:
        {
            imshow("src_img",src_img);
        }
        break;
    /**-Empty mode-**/
    default:
        imshow("src_img",src_img);
        /**-Empty mode-**/
        break;
    }
    #ifdef CAMERA
     cap.cameraReleasebuff();
    #endif
}

bool RM_Vision_Init::is_continue()
{
    bool go_on=false;
    int key =waitKey(0);
    if((char)key== 32)
    {
        go_on=true;
    }
    else
    {
        go_on=false;
    }
    return go_on;
}

bool RM_Vision_Init::is_exit()
{
    bool exit = false;
    int key = waitKey(1);
    if((char)key == 27)
    {
        exit = true;
    }
    else
    {
        exit = false;
    }
    return exit;
}

