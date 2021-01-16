#include "control/rm_link.h"

int main()
{
    RM_Vision_Init run;
#if IS_SERIAL_OPEN == 1
    SerialPort serialport;
#endif
    Fps fps;

//    g_Ctrl.my_color = ALL_COLOR;
//    g_Ctrl.now_run_mode = DEFAULT_MODE;
    for(;;)
    {
        fps.starttheTime();
        /** run **/
        run.Run();
        fps.endtheTime();
        run.armor._t = fps.time;
#if COUT_FPS == 1
        fps.displayframeRate();
#endif
        #if ANALYZE_EACH_FRAME == 1
        if(run.is_continue()){
            continue;
        }
        #endif

        if(run.is_exit()){
            break;
        }
    }
    return EXIT_SUCCESS;
}
