#ifndef FCNTL_H__
#define FCNTL_H__
#ifndef ssize_t
#define ssize_t	int
#endif
#define nfds_t int
struct pollfd{
	int fd;			//The following descriptor being polled.
	short events;	//The input event flags (see below).
	short revents;	//The output event flags (see below).
};

#define O_RDONLY	0x0000
#define O_WRONLY	0x0001
#define O_RDWR		0x0002

#define POLLIN          0x0001	//Data other than high-priority data may be read without blocking.
#define POLLPRI         0x0002	//High priority data may be read without blocking.
#define POLLOUT         0x0004	//Normal data may be written without blocking.
#define POLLERR         0x0008	//An error has occurred ( revents only).
#define POLLHUP         0x0010	//Device has been disconnected ( revents only).
#define POLLNVAL        0x0020	//Invalid fd member ( revents only).

int 	open_dev(const char *pathname, int flags);
int 	close	(int fd);
ssize_t read_dev(int fd, void *buf, size_t count);
ssize_t write	(int fd, const void *buf, size_t count);
int 	ioctl	(int d, int request, ...);
int 	poll	(struct pollfd *fds, nfds_t nfds, int timeout);
#endif
