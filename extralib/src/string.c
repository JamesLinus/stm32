#include "string.h"
#if defined(OS_FREERTOS)
int     strncmp_s(const char *dst, const char *src, size_t siz){
	while(siz > 0){
		if(*dst == '\0' && *src != '\0') return -1;
		else if(*dst != '\0' && *src == '\0') return 1;
		else if(*dst == '\0' && *src == '\0') return 0;
		else if(*dst > *src) return 1;
		else if(*dst < *src) return -1;

		dst++;
		src++;
		siz--;
	}
	return 0;
}
char *	strncpy_s(char *dst, const char *src, size_t siz){
	register char *d = dst;
	register const char *s = src;
	register size_t n = siz;
	if(!dst || !src) return 0;

	/* Copy as many bytes as will fit */
	if (n != 0 && --n != 0) {
		do {
			if ((*d++ = *s++) == 0)
				break;
		} while (--n != 0);
	}

	/* Not enough room in dst, add NUL and traverse rest of src */
	if (n == 0) {
		if (siz != 0)
			*d = '\0';		/* NUL-terminate dst */
		while (*s++)
			;
	}

	return (char*)(s - src - 1);	/* count does not include NUL */
}
size_t  strlen_s(const char *s){
    const char * end = 0;
    if(!s) return 0;
    end = (const char *)memchr(s, '\0', LIB_STRING_MAX_LENGTH);
    if (end == 0)
        return 0;
    else
        return end - s;
}
void *	memset(void *s, int val, size_t count){
	char* p = (char*)s;
	while(count > 0){
		*p = val;
		p++;
		count --;
	}
	return s;
}
void *	memcpy_s(void* __dest, const void *__src, size_t count, size_t dest_size){
	int i = 0;
	const unsigned char*__restrict s = (const unsigned char*__restrict)__src;
	unsigned char*__restrict d = (unsigned char*__restrict)__dest;

	if(count > dest_size){
		count = dest_size;
	}
	for(i = 0; i < count; i++){
		d[i] = s[i];
	}
	return __dest;
}
#endif
//end of file
