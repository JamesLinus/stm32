#include "drv_api.h"
#include "drv_errno.h"
#include <string.h>


int      errno = 0;
struct platform_driver* g_list_drivers	= 0;
extern init_fxn		___drv_init_begin;
extern init_fxn		___drv_init_end;

struct platform_driver *drv_getDriverFromName(const char* name){
	struct platform_driver *drv = 0;
	struct platform_driver *p 	= g_list_drivers;
	while(p){
		if(strcmp(p->driver.name, name) == 0){
			drv = p;
			break;
		}
		p = p->next;
	}
	return drv;
}

int platform_driver_register(struct platform_driver *driver){
	int ret = -EPERM;
	struct platform_driver *drv = 0;

	drv = g_list_drivers;
	if(drv){
		while(drv->next)
			drv = drv->next;
		drv->next = driver;
		ret = 0;
	}else{
		g_list_drivers = driver;
		ret = 0;
	}
	driver->next = 0;
	return ret;
}
int platform_device_register(struct platform_device *pdev){
	int ret = -EPERM;
	struct platform_driver *drv = 0;
	struct platform_device *p = 0;

	drv = drv_getDriverFromName(pdev->name);
	if(drv){
		p = drv->driver.devices;
		if(p){
			while(p->next) p = p->next;
			p->next = pdev;
		}else{
			drv->driver.devices = pdev;
		}
		pdev->next = 0;
		ret = 0;
	}
	return ret;
}

int drv_probe(){
	init_fxn *elem;
    elem = &___drv_init_begin;
    while(elem < &___drv_init_end){
        (*elem)();
        elem++;
    }
    return 0;
}
int open(const char *pathname, int flags){
	int ret = -EPERM;
	struct platform_driver *drv = g_list_drivers;
	struct platform_device *pdev = 0;
	int found = 0;
	int drv_index = 0;
	int dev_index = 0;

	if(!g_list_drivers) return ret;

	while(drv && !found){
		dev_index = 0;
		pdev = drv->driver.devices;
		while(pdev && !found){
			if(strcmp(pdev->dev_name, pathname) == 0){
				ret = ((((uint16_t)(drv_index)) & 0x00FF) << 8) | (((uint16_t)dev_index) & 0x00FF);
				found = 1;
				break;
			}
			pdev = pdev->next;
			dev_index++;
		}
		drv = drv->next;
		drv_index++;
	}
	if(found){
		if((found = drv->open(pdev, flags)) < 0)
			ret = found;
	}
	return ret;
}
//end of file
