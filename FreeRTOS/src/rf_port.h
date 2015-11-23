#ifndef RF_PORT_H__
#define RF_PORT_H__
#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

struct mrf24j40_device{
    int fd_spi;
    int fd_cs;
    int fd_reset;
    int fd_intr;
    int fd_wake;
};

unsigned int rf_port_get_sys_count();

void rf_port_set_if();
void rf_port_clear_if();
int  rf_port_get_if();
void rf_port_enable_intr(int enable);
int  rf_port_get_intr_enable();
void rf_port_cs(int val);
void rf_port_push_data(unsigned char val);
unsigned char rf_port_get_data();
void rf_port_reset(int val);
int  rf_port_get_intr_pin();

extern void drv_mrf_handler(void);    

#endif
