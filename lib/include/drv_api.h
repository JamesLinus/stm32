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
#include <fcntl.h>

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
	int		(*write)	(struct platform_device *device, const void* buf, int count);
	int 	(*ioctl)	(struct platform_device *device, int request, unsigned int arguments);
	/* @retval > 0 success
	 * @retval = 0 timeout
	 * @retval < 0 error
	 */
	int		(*select)	(struct platform_device *device, int *readfs, int *writefd, int *exceptfd, int timeout);

	struct device_driver 	driver;
	void					*archdata;
	struct platform_driver	*next;		//pointer to next driver
};

int driver_probe();
int platform_driver_register(struct platform_driver *driver);	// register a driver with kernel
int platform_device_register(struct platform_device *pdev);		// register a device with kernel

//#define offsetof(TYPE, MEMBER) ((int) &((TYPE *)0)->MEMBER)
//#define container_of(ptr, type, member) (type *)(((int)ptr) - offsetof(type, member))
//#define DRV_REGISTER(drv) struct platform_driver* drv_##drv __attribute__((__section__(".drv"))) = (struct platform_driver*)&drv
#define module_init(fxn) init_fxn drv_init_fx_##fxn __attribute__((__section__(".drv_init"))) = (init_fxn)&fxn

#ifdef	__cplusplus
}
#endif

#endif	/* DRV_API_H */

