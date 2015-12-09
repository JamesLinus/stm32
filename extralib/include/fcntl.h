#ifndef __FCNTL_H__
#define __FCNTL_H__
#include <stddef.h>
#include <stdint.h>
#undef fd_set
typedef struct{
	int fd;
}fd_set;
#define FD_INVALID	(0xFFFFFFFF)
#undef timeval
struct timeval {
	long	tv_sec;		/* seconds */
	long	tv_usec;	/* and microseconds */
};
#undef O_RDONLY
#define O_RDONLY	0x0000
#undef O_WRONLY
#define O_WRONLY	0x0001
#undef O_RDWR
#define O_RDWR		0x0002

int 	open_dev(const char *pathname, int flags);
#undef close
int 	close	(int fd);
int		read_dev(int fd, void *buf, size_t count);
#undef write
int		write	(int fd, const void *buf, size_t count);
#undef ioctl
int 	ioctl	(int d, int request, unsigned int arguments);
/*On  success,  select()  and pselect() return the number of 
 * file descriptors contained in the three returned descriptor sets 
 * (that is, the total number of bits that are set in readfds, writefds, exceptfds) 
 * which may be zero if the timeout expires before anything interesting happens.
 * On error, -1 is returned, and errno is set appropriately; 
 * the sets and timeout become undefined, so do not rely on their contents after an error.
 */ 
#undef select
int 	select(int nfds, fd_set *readfds, fd_set *writefds,
		  fd_set *exceptfds, struct timeval *timeout);
#undef FD_CLR
void 	FD_CLR	(int fd, fd_set *set);
#undef FD_ISSET
int  	FD_ISSET(int fd, fd_set *set);
#undef FD_SET
void 	FD_SET	(int fd, fd_set *set);
#undef FD_ZERO
void 	FD_ZERO	(fd_set *set);
#endif
