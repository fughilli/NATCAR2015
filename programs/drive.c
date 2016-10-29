#ifdef __cplusplus
extern "C" {
#endif

#include "drive.h"
#include "utils/fast_utils.h"
#include "programs/shell/shell.h"
#include "drivers/motor/driver_motor.h"

#include <stdint.h>
#include <stdbool.h>

int drive_main(char* argv[], int argc)
{
    char printbuf[64];
    if(argc != 2)
    {
        return -1;
    }

    bool succ = false;
    float val = fast_sntof(argv[1], fast_strlen(argv[1]), 10, &succ);

    if(succ)
    {
        motor_setSpeedf(val);
        fast_snprintf(printbuf, 64, "Speed=%d.%05d\r\n", (long)val,(long)(100000*(val - (long)val)));
    }
    else
    {
        shell_stream_adapter.puts("Invalid argument!\r\n");
        return -1;
    }

    shell_stream_adapter.puts(printbuf);

    return 0;
}

#ifdef __cplusplus
}
#endif
