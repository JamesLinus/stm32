#ifndef LIB_DEFINES_H__
#define LIB_DEFINES_H__
#include <stdint.h>
#include <stddef.h>
#define LIB_STRING_MAX_LENGTH	1024

void _lib_memcpy(void* dest, const void* src, int len);
#endif
