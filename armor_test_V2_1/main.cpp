#include "rm_link.h"

int main()
{
    RM_Vision_Init run;
    /** SerialPort Srart**/



    for(;;)
    {
        g_runtime = double(getTickCount());
        /** run **/
        run.Run();

        g_runtime = ((double)getTickCount() - g_runtime) / getTickFrequency();

#if COUT_FPS == 1
        int fps = int(1.0 / g_runtime);
        cout<< endl << "FPS: " << fps<< endl;
#endif

        if(run.is_exit())
        {
            break;
        }
//        if(run.is_continue())
//        {
//            continue;
//        }
    }

    return 1;
}
