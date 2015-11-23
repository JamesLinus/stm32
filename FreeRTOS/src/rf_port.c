#include "rf_port.h"
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <semaphore.h>
#include <mqueue.h>
#include <spidev.h>

volatile int g_rf_if_flag = 0;

extern struct mrf24j40_device g_rf_dev;

unsigned int rf_port_get_sys_count(){
    return xTaskGetTickCount();
}
// clear external interrupt flag
void rf_port_clear_if(){
    g_rf_if_flag = 0;
}
// enable external interrupt
void rf_port_enable_intr(int enable){
}
// get enable externla interrupt
int  rf_port_get_intr_enable(){
    return 1;
}
// select spi
void rf_port_cs(int val){
    uint8_t u8val = val;
    write(g_rf_dev.fd_cs, &u8val, 1);
}
// put data
void rf_port_push_data(unsigned char val){
    uint8_t tx[1];
    struct spi_ioc_transfer xfer;
    tx[0] = val;
    xfer.tx_buf = (unsigned int)tx;
    xfer.rx_buf = 0;
    xfer.len    = 1;
    xfer.bits_per_word  = 0;// Unchange
    xfer.speed_hz       = 0;// Unchanged
    ioctl(g_rf_dev.fd_spi, SPI_IOC_MESSAGE(1), (unsigned int)&xfer);
}
unsigned char rf_port_get_data(){
    uint8_t tx[1];
    struct spi_ioc_transfer xfer;
    xfer.tx_buf = 0;
    xfer.rx_buf = (unsigned int)tx;
    xfer.len    = 1;
    tx[0] = 0;
    ioctl(g_rf_dev.fd_spi, SPI_IOC_MESSAGE(1), (unsigned int)&xfer);
    return tx[0];
}
void rf_port_reset(int val){
    uint8_t u8val = val;
    write(g_rf_dev.fd_reset, &u8val, 1);
}
int  rf_port_get_intr_pin(){
    return g_rf_if_flag;
}
void rf_port_set_if(){
    g_rf_if_flag = 1;
}
int rf_port_get_if(){
    return g_rf_if_flag;
}
// end of file
