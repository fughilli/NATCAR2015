#include "batmon.h"
#include "utils/fast_utils.h"
#include "programs/shell/shell.h"

#ifdef __cplusplus
extern "C" {
#endif

int batmon_main(char* argv[], int argc)
{
    uint16_t bat_mv = batmon_get();
    char printbuf[64];

    fast_snprintf(printbuf, 64, "Bat=%dmV\r\n", (long)bat_mv);
    shell_stream_adapter.puts(printbuf);
    shell_stream_adapter.flush();

    return 0;
}

#ifdef __cplusplus
}
#endif
