/*
 * prog_getset.h
 *
 *  Created on: Apr 19, 2015
 *      Author: Kevin
 */

#ifndef PROG_GETSET_H_
#define PROG_GETSET_H_

#ifdef __cplusplus
extern "C" {
#endif

#define SET_MAX_NAMELEN (16)
#define MAX_SET_ENTRIES (64)

typedef enum
{
	VARTYPE_FLOAT,
	VARTYPE_INT,
	VARTYPE_UINT,
	VARTYPE_BOOL
} vartype_t;

typedef struct
{
	char name[SET_MAX_NAMELEN];
	vartype_t type;
	void* data;
} set_entry_t;

extern set_entry_t set_entries[MAX_SET_ENTRIES];

int get_main(char* argv[], int argc);
int set_main(char* argv[], int argc);

void set_register_variable(const char* name, vartype_t type, void* data);

void set_init_table();

#ifdef __cplusplus
}
#endif

#endif /* PROG_GETSET_H_ */
