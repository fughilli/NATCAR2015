#include "reboot.h"
#include "driverlib/sysctl.h"
#include "programs/shell/shell.h"

#ifdef __cplusplus
extern "C" {
#endif

int reboot_main(char* argv[], int argc)
{
    shell_stream_adapter.puts("Going down for reboot NOW!\r\n");
    shell_stream_adapter.flush();

    SysCtlReset();

    return 0;
}

#ifdef __cplusplus
}
#endif
