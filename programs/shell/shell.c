/*
 * shell.c
 *
 *  Created on: Mar 28, 2015
 *      Author: Kevin
 */

#include "shell.h"
#include <stdint.h>
#include "drivers/driver_serial.h"

#include "global_settings.h"

//const extern shell_stream_adapter_t shell_stream_adapter;

const uint8_t _bsp_seq[] = {SHELL_BACKSPACE, ' ', SHELL_BACKSPACE};

#define SHELL_IFS (' ')

char shell_lineBuffer[SHELL_LINE_BUFFER_SIZE];
uint32_t shell_lineBufferIndex;

char* shell_argBuffer[SHELL_MAX_ARGS];

const char shell_error_too_many_args[] = "Parse error: too many arguments!";
const char shell_error_unknown_command[] = "Error: unknown command \'";
const char shell_error_mismatched_quotes[] = "Error: mismatched quotes!";

void shell_processLine(void);

shell_progMapEntry_t shell_progMap[SHELL_MAX_PROGRAMS];

int help_main(char* argv[], int argc)
{
    shell_stream_adapter.puts("The following programs are available:\r\n");
    int i;
    for(i = 0; i < SHELL_MAX_PROGRAMS; i++)
    {
        if(fast_strlen(shell_progMap[i].name))
        {
            shell_stream_adapter.puts(shell_progMap[i].name);
            shell_stream_adapter.puts("\r\n");
        }
    }
    return 0;
}

void shell_initProgMap()
{
    int i;
    for(i = 0; i < SHELL_MAX_PROGRAMS; i++)
    {
        shell_progMap[i].name[0] = 0;
    }
}

void shell_init()
{
    shell_stream_adapter.puts(DEVICE_IDENTIFIER ":>");
    shell_lineBufferIndex = 0;

    shell_initProgMap();

    shell_registerProgram("help", help_main);
}

bool shell_registerProgram(const char* name, shell_func_t prog_main)
{
    if(fast_strlen(name) > SHELL_MAX_PROGRAM_NAMELEN-1)
        return false;

    int i;
    for(i = 0; i < SHELL_MAX_PROGRAMS; i++)
    {
        if(shell_progMap[i].name[0] == 0)
        {
            fast_strcpy(shell_progMap[i].name, name);
            shell_progMap[i].prog_main = prog_main;
            return true;
        }
    }

    return false;
}

shell_func_t shell_getProg(const char* name)
{
    if(fast_strlen(name) > SHELL_MAX_PROGRAM_NAMELEN-1)
        return NULL;

    int i;
    for(i = 0; i < SHELL_MAX_PROGRAMS; i++)
    {
        if(fast_strcmp(shell_progMap[i].name, name) == 0)
        {
            return shell_progMap[i].prog_main;
        }
    }

    return NULL;
}

void shell_poll()
{
    if (!shell_stream_adapter.avail(SERIAL_MODULE))
        return;

    while (shell_stream_adapter.avail(SERIAL_MODULE))
    {
        char nextChar = shell_stream_adapter.getc(SERIAL_MODULE);

        // Handle newline (end-of-command)
        if (nextChar == '\r')
        {
            shell_lineBuffer[shell_lineBufferIndex++] = 0;

            shell_stream_adapter.puts("\r\n");

            shell_processLine();

            shell_stream_adapter.puts(DEVICE_IDENTIFIER ":>");
            shell_lineBufferIndex = 0;
            break;
        }
        else if (nextChar == SHELL_BACKSPACE)
        {
            if (shell_lineBufferIndex > 0)
            {
                shell_lineBufferIndex--;
                shell_stream_adapter.writebuf(_bsp_seq, 3);
            }
            else
            {
                shell_stream_adapter.putc(SHELL_BELL_CHAR);
            }
            continue;
        }
        // Handle special character
        else if (nextChar < ' ')
        {
            continue;
        }
        else
        {
            shell_lineBuffer[shell_lineBufferIndex++] = nextChar;
        }

        // Handle line buffer overrun
        if (shell_lineBufferIndex == SHELL_LINE_BUFFER_SIZE)
        {
            shell_stream_adapter.putc(SHELL_BELL_CHAR);
            shell_lineBufferIndex--;
            continue;
        }

        shell_stream_adapter.putc(nextChar);
    }
}

typedef struct
{
    shell_func_t func;
    char** argv;
    int argc;
} shell_run_shim_params_t;

void shell_processLine(void)
{
    // Initial term is at start of line
    char* shell_startOfTerm = shell_lineBuffer;

    char quoteChar = 0;

    // Find end of term (delimited on IFS)
    uint32_t shell_lineIndex, shell_argIndex = 0;
    for (shell_lineIndex = 0; shell_lineIndex < shell_lineBufferIndex;
            shell_lineIndex++)
    {
        char nextChar = shell_lineBuffer[shell_lineIndex];

        if(quoteChar)
        {
            bool breakOnQuote = false;
            for (; shell_lineIndex < shell_lineBufferIndex;
                        shell_lineIndex++)
                {
                    nextChar = shell_lineBuffer[shell_lineIndex];
                    if(nextChar == quoteChar)
                    {
                        breakOnQuote = true;
                        break;
                    }
                }

            if(!breakOnQuote)
            {
                shell_stream_adapter.puts(shell_error_mismatched_quotes);
                return;
            }
        }

        switch (nextChar)
        {
        case '\'':
        case '\"':
            if(!quoteChar)
            {
                quoteChar = nextChar;
                shell_startOfTerm = &shell_lineBuffer[shell_lineIndex + 1];
                break;
            }
            else if(nextChar == quoteChar)
            {
                quoteChar = 0;
                shell_lineBuffer[shell_lineIndex] = 0;
                break;
            }
        case SHELL_IFS:
        case 0:
            // If the maximum argument count has been reached
            if (shell_argIndex == SHELL_MAX_ARGS)
            {
                shell_stream_adapter.puts(shell_error_too_many_args);
                return;
            }

            // Swap the space character with a null byte
            shell_lineBuffer[shell_lineIndex] = 0;
            // Add the just-completed term to the argument list
            shell_argBuffer[shell_argIndex++] = shell_startOfTerm;
            // Start the next term after the null byte
            while(shell_lineBuffer[shell_lineIndex + 1] == SHELL_IFS && shell_lineIndex != shell_lineBufferIndex)
                shell_lineIndex++;
            shell_startOfTerm = &shell_lineBuffer[shell_lineIndex + 1];
            break;
        }
    }

    shell_run_shim_params_t shim_params;
    if((shim_params.func = shell_getProg(shell_argBuffer[0])))
    {
        //shim_params.argc = shell_argIndex;
        //shim_params.argv = shell_argBuffer;

        shim_params.func(shell_argBuffer, shell_argIndex);
    }
    else
    {
        shell_stream_adapter.puts(shell_error_unknown_command);
        shell_stream_adapter.puts(shell_argBuffer[0]);
        shell_stream_adapter.puts("\'\r\n");
    }
}

