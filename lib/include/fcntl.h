#ifndef FCNTL_H__
#define FCNTL_H__
#include <stddef.h>
#include <stdint.h>
#ifndef ssize_t
#define ssize_t	int
#endif
#define fd_set unsigned int
#define FD_INVALID	((fd_set)-1)
struct timeval {
	long	tv_sec;		/* seconds */
	long	tv_usec;	/* and microseconds */
};

#define O_RDONLY	0x0000
#define O_WRONLY	0x0001
#define O_RDWR		0x0002

int 	open_dev(const char *pathname, int flags);
int 	close	(int fd);
ssize_t read_dev(int fd, void *buf, size_t count);
ssize_t write	(int fd, const void *buf, size_t count);
int 	ioctl	(int d, int request, unsigned int arguments);
/*On  success,  select()  and pselect() return the number of 
 * file descriptors contained in the three returned descriptor sets 
 * (that is, the total number of bits that are set in readfds, writefds, exceptfds) 
 * which may be zero if the timeout expires before anything interesting happens.
 * On error, -1 is returned, and errno is set appropriately; 
 * the sets and timeout become undefined, so do not rely on their contents after an error.
 */ 
int 	select(int nfds, fd_set *readfds, fd_set *writefds,
		  fd_set *exceptfds, struct timeval *timeout);

void 	FD_CLR	(int fd, fd_set *set);
int  	FD_ISSET(int fd, fd_set *set);
void 	FD_SET	(int fd, fd_set *set);
void 	FD_ZERO	(fd_set *set);
#endif
