/* 
 * File:   lib_string.h
 * Author: dev
 *
 * Created on October 19, 2015, 1:53 PM
 */

#ifndef LIB_STRING_H
#define	LIB_STRING_H
#include "lib_defines.h"
#ifdef	__cplusplus
extern "C" {
#endif
int     strncmp_s(const char *s1, const char *s2, size_t count);
char *	strncpy_s(char *dest, const char *src, size_t count);
size_t  strlen_s(const char *s);
void *	memset(void *s, int val, size_t count);
void *	memcpy_s(void *dest, const void* src, size_t count, size_t dest_size);
#ifdef	__cplusplus
}
#endif

#endif	/* LIB_STRING_H */

