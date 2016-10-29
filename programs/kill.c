#include "kill.h"
#include "utils/fast_utils.h"
#include "programs/shell/shell.h"
#include "drivers/motor/driver_motor.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

int kill_main(char* argv[], int argc)
{
    killswitch = true;
    speed_control = false;
    motor_setSpeedf(0.0f);
    motor_brake();

    if(argc == 2 && fast_strcmp(argv[1], "unkill") == 0)
    {
        killswitch = false;
        motor_releaseBrake();
        shell_stream_adapter.puts("Released brake.\r\n");
    }
    else
    {
        shell_stream_adapter.puts("KILLSWITCH ENGAGED!\r\n");
    }

    return 0;
}

#ifdef __cplusplus
}
#endif
