#include "control/rm_link.h"

int main() {
  RM_Vision_Init run;
#if IS_SERIAL_OPEN == 1
  SerialPort serialport;
#endif
  //    g_Ctrl.my_color = ALL_COLOR;
  //    g_Ctrl.now_run_mode = DEFAULT_MODE;
  
    /** run **/
    run.Run();

  return EXIT_SUCCESS;
}
