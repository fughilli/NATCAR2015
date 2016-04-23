/*
 * prog_getset.c
 *
 *  Created on: Apr 19, 2015
 *      Author: Kevin
 */

#include "prog_getset.h"
#include "../fast_utils.h"
#include "../driver_serial.h"
#include <stdbool.h>

#include "global_settings.h"

void set_register_variable(const char* name, vartype_t type, void* data)
{
	int i;
	for(i = 0; i < MAX_SET_ENTRIES; i++)
	{
		if(fast_strlen(set_entries[i].name) == 0)
		{
			fast_memcpy(set_entries[i].name, name, fast_strlen(name) + 1);
			set_entries[i].data = data;
			set_entries[i].type = type;
			return;
		}
	}
}

void set_init_table()
{
	int i;
	for (i = 0; i < MAX_SET_ENTRIES; i++)
	{
		set_entries[i].name[0] = 0;
	}
}

int set_main(char* argv[], int argc)
{
	char printbuf[64];
	if(argc != 3)
	{
		Serial_puts(SERIAL_MODULE, "USAGE:\r\n\tset <VAR> <VAL>\r\n");
		return -1;
	}

	int i;

	bool succ;

	for(i = 0; i < MAX_SET_ENTRIES; i++)
	{
		if(fast_strlen(set_entries[i].name) && fast_strcmp(argv[1], set_entries[i].name) == 0)
		{
			switch(set_entries[i].type)
			{
			case VARTYPE_FLOAT:
				(*((float*)(set_entries[i].data))) = fast_sntof(argv[2], fast_strlen(argv[2]), 10, &succ);
				break;
			case VARTYPE_INT:
				(*((int32_t*)(set_entries[i].data))) = fast_sntol(argv[2], fast_strlen(argv[2]), 10, &succ);
				break;
			case VARTYPE_UINT:
				(*((uint32_t*)(set_entries[i].data))) = fast_sntoul(argv[2], fast_strlen(argv[2]), 10, &succ);
				break;
			case VARTYPE_BOOL:
				(*((bool*)(set_entries[i].data))) = fast_sntob(argv[2], fast_strlen(argv[2]), &succ);
			}

			if(!succ)
			{
				Serial_puts(SERIAL_MODULE, "Invalid value!\r\n");
				return -1;
			}

			switch(set_entries[i].type)
			{
			case VARTYPE_FLOAT:
				Serial_puts(SERIAL_MODULE, "float -> ");
				break;
			case VARTYPE_INT:
				Serial_puts(SERIAL_MODULE, "int -> ");
				break;
			case VARTYPE_UINT:
				Serial_puts(SERIAL_MODULE, "uint -> ");
				break;
			case VARTYPE_BOOL:
				Serial_puts(SERIAL_MODULE, "bool -> ");
				break;
			}

			Serial_puts(SERIAL_MODULE, set_entries[i].name);

			float ffloat;
			int32_t fint;
			uint32_t fuint;
			bool fbool;

			switch (set_entries[i].type)
			{
			case VARTYPE_FLOAT:
				ffloat = (*((float*)(set_entries[i].data)));
				fast_memcpy(printbuf, " := ", 4);
				fint = fast_snfmtf(printbuf + 4, 60, ffloat, 6, 10);
				fast_memcpy(printbuf + 4 + fint, "\r\n", 3);
				break;
			case VARTYPE_INT:
				fint = (*((int32_t*)(set_entries[i].data)));
				fast_snprintf(printbuf, 64, " := %d\r\n", fint);
				break;
			case VARTYPE_UINT:
				fuint = (*((uint32_t*)(set_entries[i].data)));
				fast_snprintf(printbuf, 64, " := %ud\r\n", fuint);
				break;
			case VARTYPE_BOOL:
				fbool = (*((bool*)(set_entries[i].data)));
				fast_memcpy(printbuf, " := ", 4);
				if(fbool)
					fast_memcpy(printbuf + 4, "true\r\n", 7);
				else
					fast_memcpy(printbuf + 4, "false\r\n", 8);
				break;

			}

			Serial_puts(SERIAL_MODULE, printbuf);

			return 0;
		}
	}

	Serial_puts(SERIAL_MODULE, "No such variable: ");
	Serial_puts(SERIAL_MODULE, argv[1]);
	Serial_puts(SERIAL_MODULE, "\r\n");

//	bool succ = false, succui = false;
//	float val = fast_sntof(argv[2], fast_strlen(argv[2]), 10, &succ);
//	uint32_t valui = fast_sntoul(argv[2], fast_strlen(argv[2]), 10, &succui);
//
//	if (succ)
//	{
//		if (fast_strcmp(argv[1], "P") == 0 || fast_strcmp(argv[1], "p") == 0)
//		{
//			servopid.coefficients.p = val;
//			fast_snprintf(printbuf, 64, "P=%d.%05d\r\n",
//					(long) servopid.coefficients.p,
//					(long) (100000
//							* (servopid.coefficients.p
//									- (long) servopid.coefficients.p)));
//		}
//		if (fast_strcmp(argv[1], "I") == 0 || fast_strcmp(argv[1], "i") == 0)
//		{
//			servopid.coefficients.i = val;
//			fast_snprintf(printbuf, 64, "I=%d.%05d\r\n",
//					(long) servopid.coefficients.i,
//					(long) (100000
//							* (servopid.coefficients.i
//									- (long) servopid.coefficients.i)));
//		}
//		if (fast_strcmp(argv[1], "D") == 0 || fast_strcmp(argv[1], "d") == 0)
//		{
//			servopid.coefficients.d = val;
//			fast_snprintf(printbuf, 64, "D=%d.%05d\r\n",
//					(long) servopid.coefficients.d,
//					(long) (100000
//							* (servopid.coefficients.d
//									- (long) servopid.coefficients.d)));
//		}
//		if (fast_strcmp(argv[1], "thresh") == 0)
//		{
//			threshold_ratio = val;
//			fast_snprintf(printbuf, 64, "D=%d.%05d\r\n",
//					(long) threshold_ratio,
//					(long) (100000
//							* (threshold_ratio
//									- (long) threshold_ratio)));
//		}
//	}
//
//	if(succui)
//	{
//		if (fast_strcmp(argv[1], "smaxb") == 0)
//		{
//			servo_max_highband = valui;
//			fast_snprintf(printbuf, 64, "servo_max_highband=%d\r\n", valui);
//		}
//		if (fast_strcmp(argv[1], "sminb") == 0)
//		{
//			servo_min_highband = valui;
//			fast_snprintf(printbuf, 64, "servo_min_highband=%d\r\n", valui);
//		}
//	}
//
//	if(!(succ || succui))
//	{
//		Serial_puts(SERIAL_MODULE, "Invalid arguments!\r\n", 100);
//		return -1;
//	}
//
//	Serial_puts(SERIAL_MODULE, printbuf, 64);
	return -1;
}

int get_main(char* argv[], int argc)
{
	char printbuf[64] = "No arg specified!\r\n";
	if(argc != 2)
	{
		Serial_puts(SERIAL_MODULE, "USAGE:\r\n\tget <VAR>\r\n");
		return -1;
	}

	int i;

	bool listall = false;

	if(fast_memcmp(argv[1], "list", 4) == 0)
	{
		listall = true;
	}

	for(i = 0; i < MAX_SET_ENTRIES; i++)
	{
		if(fast_strlen(set_entries[i].name) && (listall || (fast_strcmp(argv[1], set_entries[i].name) == 0)))
		{
			switch(set_entries[i].type)
			{
			case VARTYPE_FLOAT:
				Serial_puts(SERIAL_MODULE, "float -> ");
				break;
			case VARTYPE_INT:
				Serial_puts(SERIAL_MODULE, "int -> ");
				break;
			case VARTYPE_UINT:
				Serial_puts(SERIAL_MODULE, "uint -> ");
				break;
			case VARTYPE_BOOL:
				Serial_puts(SERIAL_MODULE, "bool -> ");
				break;
			}

			Serial_puts(SERIAL_MODULE, set_entries[i].name);

			float ffloat;
			int32_t fint;
			uint32_t fuint;
			bool fbool;

			switch (set_entries[i].type)
			{
			case VARTYPE_FLOAT:
				ffloat = (*((float*)(set_entries[i].data)));
				//fast_snprintf(printbuf, 64, " := %f\r\n", ffloat);
				fast_memcpy(printbuf, " := ", 4);
				fint = fast_snfmtf(printbuf + 4, 60, ffloat, 6, 10);
				fast_memcpy(printbuf + 4 + fint, "\r\n", 3);
				break;
			case VARTYPE_INT:
				fint = (*((int32_t*)(set_entries[i].data)));
				fast_snprintf(printbuf, 64, " := %d\r\n", fint);
				break;
			case VARTYPE_UINT:
				fuint = (*((uint32_t*)(set_entries[i].data)));
				fast_snprintf(printbuf, 64, " := %ud\r\n", fuint);
				break;
			case VARTYPE_BOOL:
				fbool = (*((bool*) (set_entries[i].data)));
				fast_memcpy(printbuf, " := ", 4);
				if (fbool)
					fast_memcpy(printbuf + 4, "true\r\n", 7);
				else
					fast_memcpy(printbuf + 4, "false\r\n", 8);
				break;
			}

			Serial_puts(SERIAL_MODULE, printbuf);

			if(!listall)
				return 0;
		}
	}

	if(listall)
		return 0;

	Serial_puts(SERIAL_MODULE, "No such variable: ");
	Serial_puts(SERIAL_MODULE, argv[1]);
	Serial_puts(SERIAL_MODULE, "\r\n");

//	if(argc == 2)
//	{
//		if (fast_strcmp(argv[1], "P") == 0 || fast_strcmp(argv[1], "p") == 0)
//		{
//			fast_snprintf(printbuf, 64, "P=%d.%05d\r\n", (long)servopid.coefficients.p,(long)(100000*(servopid.coefficients.p - (long)servopid.coefficients.p)));
//		}
//		if (fast_strcmp(argv[1], "I") == 0 || fast_strcmp(argv[1], "i") == 0)
//		{
//			fast_snprintf(printbuf, 64, "I=%d.%05d\r\n", (long)servopid.coefficients.i,(long)(100000*(servopid.coefficients.i - (long)servopid.coefficients.i)));
//		}
//		if (fast_strcmp(argv[1], "D") == 0 || fast_strcmp(argv[1], "d") == 0)
//		{
//			fast_snprintf(printbuf, 64, "D=%d.%05d\r\n", (long)servopid.coefficients.d,(long)(100000*(servopid.coefficients.d - (long)servopid.coefficients.d)));
//		}
//		if (fast_strcmp(argv[1], "thresh") == 0)
//		{
//			fast_snprintf(printbuf, 64, "threshold_ratio=%d.%05d\r\n", (long)threshold_ratio,(long)(100000*(threshold_ratio - (long)threshold_ratio)));
//		}
//		if (fast_strcmp(argv[1], "smaxb") == 0)
//		{
//			fast_snprintf(printbuf, 64, "servo_max_highband=%d\r\n",
//					servo_max_highband);
//		}
//		if (fast_strcmp(argv[1], "sminb") == 0)
//		{
//			fast_snprintf(printbuf, 64, "servo_min_highband=%d\r\n",
//					servo_min_highband);
//		}
//		Serial_puts(SERIAL_MODULE, printbuf, 64);
//	}

	return -1;
}

