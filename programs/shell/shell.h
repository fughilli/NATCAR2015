/*
 * shell.h
 *
 *  Created on: Mar 28, 2015
 *      Author: Kevin
 */

#ifndef SHELL_H_
#define SHELL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "osprogram.h"
#include "../fast_utils.h"
#include <stdbool.h>

#define DEVICE_IDENTIFIER "DankWheels"

#define SHELL_LINE_BUFFER_SIZE (128)

#define SHELL_MAX_ARGS (8)
#define SHELL_MAX_PROGRAMS (16)
#define SHELL_MAX_PROGRAM_NAMELEN (16)

#define SHELL_BELL_CHAR (7)
#define SHELL_BACKSPACE (8)

typedef int(*shell_func_t)(char* argv[], int argc);

typedef struct
{
    int  (*getc)();
    void (*putc)(char c);
    void (*puts)(const char* s);
    void (*writebuf)(const uint8_t* buf, uint32_t len);
    void (*flush)();
    bool (*avail)();
} shell_stream_adapter_t;

extern const shell_stream_adapter_t shell_stream_adapter;

typedef struct
{
    char name[SHELL_MAX_PROGRAM_NAMELEN];
    shell_func_t prog_main;
} shell_progMapEntry_t;

void shell_poll();
void shell_init();
bool shell_registerProgram(const char* name, shell_func_t prog_main);

#ifdef __cplusplus
}
#endif

#endif /* SHELL_H_ */
