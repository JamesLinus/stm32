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
#if defined(OS_FREERTOS)
int     strncmp_s(const char *s1, const char *s2, size_t count);
char *	strncpy_s(char *dest, const char *src, size_t count);
size_t  strlen_s(const char *s);
void *	memset(void *s, int val, size_t count);
void *	memcpy_s(void *dest, const void* src, size_t count, size_t dest_size);
#elif defined(OS_UCOS)
#include <os.h>
#include <lib_mem.h>
#include <lib_str.h>
#define strncmp_s(d, s, s_n)			Str_Cmp_N(d, s, s_n)
#define strncpy_s(d, s, s_n)			Str_Copy_N(d, s, s_n)
#define strlen_s(s)						Str_Len_N(s, 4096)

#define memcpy_s(d, s, s_n, d_n)		Mem_Copy(d, s, s_n)
#define memset(s, v, c)					Mem_Set(s, v, c)
#endif
#ifdef	__cplusplus
}
#endif

#endif	/* LIB_STRING_H */

