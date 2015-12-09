#ifndef SPIDEV_H
#define SPIDEV_H
#include <stddef.h>
#include <stdint.h>

#define SPI_CPHA_		0x01
#define SPI_CPOL_		0x02

#define SPI_MODE_0		(0|0)
#define SPI_MODE_1		(0|SPI_CPHA_)
#define SPI_MODE_2		(SPI_CPOL_|0)
#define SPI_MODE_3		(SPI_CPOL_|SPI_CPHA_)

#define SPI_CS_HIGH		0x04
#define SPI_LSB_FIRST	0x08
#define SPI_3WIRE		0x10
#define SPI_LOOP		0x20
#define SPI_NO_CS		0x40
#define SPI_READY		0x80

/**
 * struct spi_ioc_transfer - describes a single SPI transfer
 * @tx_buf: Holds pointer to userspace buffer with transmit data, or null.
 *	If no data is provided, zeroes are shifted out.
 * @rx_buf: Holds pointer to userspace buffer for receive data, or null.
 * @len: Length of tx and rx buffers, in bytes.
 * @speed_hz: Temporary override of the device's bitrate.
 * @bits_per_word: Temporary override of the device's wordsize.
 * @delay_usecs: If nonzero, how long to delay after the last bit transfer
 *	before optionally deselecting the device before the next transfer.
 * @cs_change: True to deselect device before starting the next transfer.
 *
 *	struct spi_ioc_transfer mesg;
 *	...
 *	status = ioctl(fd, SPI_IOC_MESSAGE(1), &mesg);
 */
struct spi_ioc_transfer {
	unsigned int		tx_buf;
	unsigned int		rx_buf;

	unsigned int		len;
	unsigned int		speed_hz;

	unsigned int		delay_usecs;
	unsigned char		bits_per_word;
	unsigned char		cs_change;
};

#define SPI_IOC_MESSAGE(N) 			 0x01

/* Read / Write of SPI mode (SPI_MODE_0..SPI_MODE_3) args=(unsigned int*)*/
#define SPI_IOC_RD_MODE				 0x02
#define SPI_IOC_WR_MODE				(0x02|0x80)

/* Read / Write SPI bit justification args=(unsigned int*)*/
#define SPI_IOC_RD_LSB_FIRST		 0x03
#define SPI_IOC_WR_LSB_FIRST		(0x03|0x80)

/* Read / Write SPI device word length (1..N) args=(unsigned int*)*/
#define SPI_IOC_RD_BITS_PER_WORD	 0x04
#define SPI_IOC_WR_BITS_PER_WORD	(0x04|0x80)

/* Read / Write SPI device default max speed hz args=(unsigned int*)*/
#define SPI_IOC_RD_MAX_SPEED_HZ		 0x05
#define SPI_IOC_WR_MAX_SPEED_HZ		(0x05|0x80)

#define SPI_MODULE_COUNT		(3)

struct spi_platform_data{
	uint8_t sck_pin;
	uint8_t mosi_pin;
	uint8_t miso_pin;
	uint8_t ss_pin;
#if defined(OS_FREERTOS)
	void* 		__drv_base;
#elif defined(OS_UCOS)

#endif
};


#endif /* SPIDEV_H */
