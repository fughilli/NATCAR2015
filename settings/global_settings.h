#ifndef GLOBAL_SETTINGS_H
#define GLOBAL_SETTINGS_H

//#define DEBUG

//#define SERIAL_MODULE (Serial_module_debug)
//#define SERIAL_MODULE_BAUD (115200*8)
//
#define SERIAL_MODULE (Serial_module_3)
#define SERIAL_MODULE_BAUD (1382400)

#define _CT_ATOI(val) #val
#define CT_ATOI(val) _CT_ATOI(val)

#ifdef DEBUG

#define DEBUG_LINE(_label_) Serial_puts(SERIAL_MODULE,"inside \"" _label_ "\" @ " __FILE__ ":" CT_ATOI(__LINE__) "\r\n")

#else

#define DEBUG_LINE(_label_) 0

#endif

#endif // GLOBAL_SETTINGS_H
