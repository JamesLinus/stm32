/* 
 * File:   drv_api.h
 * Author: dev
 *
 * Created on October 19, 2015, 11:20 AM
 */

#ifndef DRV_API_H
#define	DRV_API_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <stdint.h>
#include "drv_errno.h"
#define O_RDONLY	0b00000001
#define O_WRONLY	0b00000010
#define O_RDWR		(O_RDONLY | O_WRONLY)
typedef int (*init_fxn)(void);

struct device {
  void *platform_data;			// device spec
};
struct platform_device {
	const char				*dev_name;	// device name
	const char      		*name;		// name of driver is manager this device
	int             		id;			// id (index) of device
	struct device   		dev;		// device spec
	struct platform_device* next;// next device in list of drivers
 };
typedef struct pm_message {
	int event;
} pm_message_t;
struct device_driver {
	 const char              *name;		// name of driver
	 struct platform_device	 *devices;	// list of device;
};
struct platform_driver {
	int 	(*probe)	(struct platform_device *device);
	int 	(*remove)	(struct platform_device *device);
	void 	(*shutdown)	(struct platform_device *device);
	int 	(*suspend)	(struct platform_device *device, pm_message_t state);
	int 	(*resume)	(struct platform_device *device);

	int 	(*open)		(struct platform_device *device, int flags);
	int 	(*close)	(struct platform_device *device);
	int		(*read)		(struct platform_device *device, void* buf, int count);
	int		(*write)	(struct platform_device *device, void* buf, int count);
	int 	(*ioctl)	(struct platform_device *device, int request, unsigned int arguments);

	struct device_driver 	driver;
	struct platform_driver	*next;		//pointer to next driver
};
extern int errno;

int drv_probe();

int platform_driver_register(struct platform_driver *driver);	// register a driver with kernel
int platform_device_register(struct platform_device *pdev);		// register a device with kernel

int drv_initialize();
/**
 * @brief Create a new open file description, an entry in the system-wide table of open files.
 * @param pathname  Path name for a file
 * @param flags     
 * @return Return new file descriptor, -1 if an error occurred, errno is set
 */
int open(const char *pathname, int flags);
/**
 * @brief Closes a file descriptor, so that it no longer refers to any file and may be reused
 * @param fd        File descriptor
 * @return          Return new file descriptor, -1 if an error occurred, errno is set
 */
int close(int fd);
/**
 * @brief Read up to count bytes from file descriptor fd into the buffer starting at buf
 * @param fd        File descriptor
 * @param buf       Buffer
 * @param count     Count bytes
 * @return Number of bytes read, -1 if an error occurred, errno is set
 */
int read(int fd, void* buf, int count);
    /**
     * @brief Writes  up  to count bytes from the buffer pointed buf to the file referred to by the file descriptor fd.
     * @param fd        File descriptor
     * @param buf       Buffer
     * @param count     Count bytes
     * @return          Number of bytes written, -1 if an error occurred, error is set
     */
int write(int fd, void* buf, int count);
    /**
     * @brief Function manipulates the underlying device parameters of special files.
     * @param fd        File descriptor
     * @param request   Request value
     * @return Usually,  on success zero is returned. 
     *  A few ioctl() requests use the return value as an output parameter and
     *  return a nonnegative value on success.
     *  On error, -1 is returned, and errno is set appropriately.
     */ 
int ioctl(int fd, int request, unsigned int arguments);
        
//#define offsetof(TYPE, MEMBER) ((int) &((TYPE *)0)->MEMBER)
//#define container_of(ptr, type, member) (type *)(((int)ptr) - offsetof(type, member))
//#define DRV_REGISTER(drv) struct platform_driver* drv_##drv __attribute__((__section__(".drv"))) = (struct platform_driver*)&drv
#define module_init(fxn) init_fxn drv_init_fx_##fxn __attribute__((__section__(".drv_init"))) = (init_fxn)&fxn

#ifdef	__cplusplus
}
#endif

#endif	/* DRV_API_H */

