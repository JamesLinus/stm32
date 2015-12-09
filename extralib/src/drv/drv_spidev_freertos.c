#if defined(OS_FREERTOS)
#include <drv_api.h>
#include <drv_gpio.h>
#include <spidev.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "misc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
int 	spi_init		(void);
int 	spi_open		(struct platform_device *dev, int flags);
int 	spi_close		(struct platform_device *dev);
int		spi_ioctl		(struct platform_device *dev, int request, unsigned int arguments);

#define SPI_DRIVER_SPEED_CNT	8
struct spi_driver_speed_ref{
	unsigned int speed;
	unsigned char scale;
};
struct spi_driver_arch_data{
	void* rx_event[SPI_MODULE_COUNT];
	SPI_InitTypeDef 	SPI_InitStruct[SPI_MODULE_COUNT];
	struct spi_driver_speed_ref 		speed_supported[SPI_DRIVER_SPEED_CNT];
};
struct spi_driver_arch_data g_spi_driver_arch_data;

static struct platform_driver g_spi_driver = {
	.driver		= {
		.name	= "spidev-drv",
		.devices = 0,
	},
	.archdata = &g_spi_driver_arch_data,
	.probe		= 0,
	.remove		= 0,
	.resume 	= 0,
	.suspend	= 0,
	.shutdown	= 0,

	.open 		= &spi_open,
	.close 		= &spi_close,
	.read 		= 0,
	.write 		= 0,
	.ioctl 		= &spi_ioctl,
	.select		= 0,

	.next 		= 0,
};
int spi_drv_find_speed_near(unsigned int speed, int prescaler){
	int i;
	int ret = SPI_DRIVER_SPEED_CNT - 1;
	for(i = SPI_DRIVER_SPEED_CNT - 1; i >= 0; i--){
		if(g_spi_driver_arch_data.speed_supported[i].speed/prescaler > speed){
			break;
		}
		ret = i;
	}
	return ret;
}
unsigned int spi_drv_find_speed_from_scale(int scale, int prescaler){
	int i;
	unsigned int ret = 0;
	for(i = 0; i < SPI_DRIVER_SPEED_CNT; i++){
		if(g_spi_driver_arch_data.speed_supported[i].scale == scale){
			ret = g_spi_driver_arch_data.speed_supported[i].speed/prescaler;
			break;
		}
	}
	return ret;
}
int spi_init		(void){
	int i;
	memset(&g_spi_driver_arch_data, 0, sizeof(g_spi_driver_arch_data));
	g_spi_driver_arch_data.speed_supported[0].scale = SPI_BaudRatePrescaler_2;
	g_spi_driver_arch_data.speed_supported[0].speed = SystemCoreClock / 2;
	g_spi_driver_arch_data.speed_supported[1].scale = SPI_BaudRatePrescaler_4;
	g_spi_driver_arch_data.speed_supported[1].speed = SystemCoreClock / 4;
	g_spi_driver_arch_data.speed_supported[2].scale = SPI_BaudRatePrescaler_8;
	g_spi_driver_arch_data.speed_supported[2].speed = SystemCoreClock / 8;
	g_spi_driver_arch_data.speed_supported[3].scale = SPI_BaudRatePrescaler_16;
	g_spi_driver_arch_data.speed_supported[3].speed = SystemCoreClock / 16;
	g_spi_driver_arch_data.speed_supported[4].scale = SPI_BaudRatePrescaler_32;
	g_spi_driver_arch_data.speed_supported[4].speed = SystemCoreClock / 32;
	g_spi_driver_arch_data.speed_supported[5].scale = SPI_BaudRatePrescaler_64;
	g_spi_driver_arch_data.speed_supported[5].speed = SystemCoreClock / 64;
	g_spi_driver_arch_data.speed_supported[6].scale = SPI_BaudRatePrescaler_128;
	g_spi_driver_arch_data.speed_supported[6].speed = SystemCoreClock / 128;
	g_spi_driver_arch_data.speed_supported[7].scale = SPI_BaudRatePrescaler_256;
	g_spi_driver_arch_data.speed_supported[7].speed = SystemCoreClock / 256;

	platform_driver_register(&g_spi_driver);
	return 0;
}
module_init(spi_init);
//extern void LREP(char* s, ...);
int 	spi_open	(struct platform_device *dev, int flags){
	int ret;
	struct spi_platform_data* data;	
	GPIO_InitTypeDef 	GPIO_InitStruct;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	SPI_TypeDef* 		SPIx;
	int bank, pin;
	uint8_t GPIO_AF;	
	uint8_t NVIC_IRQChannel;
	
	ret = -EPERM;
	data = (struct spi_platform_data*)dev->dev.platform_data;
	
	if(dev->id < 0 || dev->id > SPI_MODULE_COUNT) return ret;
	
	switch(dev->id){
		case 0:{
			GPIO_AF = GPIO_AF_SPI1;
			SPIx = SPI1;
			NVIC_IRQChannel = SPI1_IRQn;
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
			break;
		}
		case 1:{
			GPIO_AF = GPIO_AF_SPI2;
			SPIx = SPI2;
			NVIC_IRQChannel = SPI2_IRQn;
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
			break;
		}
		case 2:{
			GPIO_AF = GPIO_AF_SPI3;
			SPIx = SPI3;
			NVIC_IRQChannel = SPI3_IRQn;
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
			break;
		}
		default:
			break;
	}
	
	// GPIO pin
	if(data->sck_pin != GPIO_PIN_INVALID){
		bank = gpio_get_bank_index(data->sck_pin);
		pin = gpio_get_pin_index(data->sck_pin);		
		RCC_AHB1PeriphClockCmd(g_gpio_bank_ref[bank].RCC_AHB1Periph_GPIOx, ENABLE);
		GPIO_InitStruct.GPIO_Pin 	= g_gpio_pin_ref[pin].GPIO_Pin;
		GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType 	= GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed 	= GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
		GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIO_InitStruct);
		GPIO_PinAFConfig(g_gpio_bank_ref[bank].GPIOx, g_gpio_pin_ref[pin].GPIO_PinSource, GPIO_AF);
	}
	if(data->mosi_pin != GPIO_PIN_INVALID){
		bank = gpio_get_bank_index(data->mosi_pin);
		pin = gpio_get_pin_index(data->mosi_pin);		
		RCC_AHB1PeriphClockCmd(g_gpio_bank_ref[bank].RCC_AHB1Periph_GPIOx, ENABLE);
		GPIO_InitStruct.GPIO_Pin 	= g_gpio_pin_ref[pin].GPIO_Pin;
		GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType 	= GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed 	= GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
		GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIO_InitStruct);
		GPIO_PinAFConfig(g_gpio_bank_ref[bank].GPIOx, g_gpio_pin_ref[pin].GPIO_PinSource, GPIO_AF);
	}
	if(data->miso_pin != GPIO_PIN_INVALID){
		bank = gpio_get_bank_index(data->miso_pin);
		pin = gpio_get_pin_index(data->miso_pin);		
		RCC_AHB1PeriphClockCmd(g_gpio_bank_ref[bank].RCC_AHB1Periph_GPIOx, ENABLE);
		GPIO_InitStruct.GPIO_Pin 	= g_gpio_pin_ref[pin].GPIO_Pin;
		GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType 	= GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed 	= GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
		GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIO_InitStruct);
		GPIO_PinAFConfig(g_gpio_bank_ref[bank].GPIOx, g_gpio_pin_ref[pin].GPIO_PinSource, GPIO_AF);
	}
	if(data->ss_pin != GPIO_PIN_INVALID){
		bank = gpio_get_bank_index(data->ss_pin);
		pin = gpio_get_pin_index(data->ss_pin);		
		RCC_AHB1PeriphClockCmd(g_gpio_bank_ref[bank].RCC_AHB1Periph_GPIOx, ENABLE);
		GPIO_InitStruct.GPIO_Pin 	= g_gpio_pin_ref[pin].GPIO_Pin;
		GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType 	= GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed 	= GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
		GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIO_InitStruct);
		GPIO_PinAFConfig(g_gpio_bank_ref[bank].GPIOx, g_gpio_pin_ref[pin].GPIO_PinSource, GPIO_AF);
		
		SPI_SSOutputCmd(SPIx, ENABLE);
	}
	/* configure SPI in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_Direction 	= SPI_Direction_2Lines_FullDuplex;
	g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_Mode 		= SPI_Mode_Master;
	g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_DataSize 	= SPI_DataSize_8b;
	g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_CPOL 		= SPI_CPOL_Low;
	g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_CPHA 		= SPI_CPHA_1Edge;
	g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_NSS 			= SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
	g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; 	// SPI frequency is APB2 frequency / 4
	g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_FirstBit 	= SPI_FirstBit_MSB;					// data is transmitted MSB first
	SPI_Init(SPIx, &g_spi_driver_arch_data.SPI_InitStruct[dev->id]); 
	
	// interrupt
	//SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, ENABLE);
	SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	SPI_Cmd(SPIx, ENABLE); // enable SPI1	
	
	data->__drv_base = SPIx;
	if(!g_spi_driver_arch_data.rx_event[dev->id])
		g_spi_driver_arch_data.rx_event[dev->id] = xQueueCreate(32, 1);
	ret = 0;
	return ret;
}
int 	spi_close	(struct platform_device *dev){
	int ret = -EPERM;
	return ret;
}
int		spi_ioctl	(struct platform_device *dev, int request, unsigned int arguments){
	int ret;
	struct spi_platform_data	*data;
	SPI_TypeDef* 		SPIx;
	struct spi_ioc_transfer	    *tr;
	unsigned char *tx, *rx, u8val;
	unsigned int  len;
	unsigned int ival;
	uint16_t sr;
	uint16_t scale_index;
	
	ret = -EPERM;
	data = (struct spi_platform_data*)dev->dev.platform_data;
	SPIx = (SPI_TypeDef*)data->__drv_base;
	
	if(dev->id < 0 || dev->id > SPI_MODULE_COUNT) return ret;
	
	switch(request){
		case SPI_IOC_MESSAGE(1):{
			tr = (struct spi_ioc_transfer*)arguments;
			len = tr->len;
			tx = (unsigned char*)tr->tx_buf;
			rx = (unsigned char*)tr->rx_buf;
			ret = 0;
			ival = 0;
			if(tr->speed_hz > 0){
				scale_index = spi_drv_find_speed_near(tr->speed_hz, (dev->id == 0) ? 4 : 2);
				g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_BaudRatePrescaler = g_spi_driver_arch_data.speed_supported[scale_index].scale;
				ival = 1;
			}
			if(tr->bits_per_word > 0){
				g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_DataSize = (tr->bits_per_word == 16) ? SPI_DataSize_16b : SPI_DataSize_8b;
				ival = 1;
			}
			if(ival)
				SPI_Init(SPIx, &g_spi_driver_arch_data.SPI_InitStruct[dev->id]);
			while(len > 0){
				if(tx){
					SPIx->DR = *tx;
					tx++;
				}
				else
					SPIx->DR = 0;
				if(xQueueReceive(g_spi_driver_arch_data.rx_event[dev->id], &u8val, portTICK_PERIOD_MS * 100) != pdTRUE)
					break;
				if(rx){
					*rx = u8val;
					rx++;
				}
				len --;
				ret ++;
			}
			break;
		}
		case SPI_IOC_WR_MODE:{
			ival = *((unsigned int*)arguments);
			if(ival & 0x00){
				g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_CPOL 	= SPI_CPOL_Low;
				g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_CPHA 	= SPI_CPHA_1Edge;

			}
			if(ival & 0x01){
				g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_CPOL 	= SPI_CPOL_High;
			}
			if(ival & 0x02){
				g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_CPHA 	= SPI_CPHA_2Edge;
			}
			if(ival & SPI_NO_CS){
				SPI_SSOutputCmd(SPIx, DISABLE);
			}
			if(ival & SPI_LSB_FIRST){
				g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_FirstBit 	= SPI_FirstBit_LSB;
			}else{
				g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_FirstBit 	= SPI_FirstBit_MSB;
			}
			SPI_Init(SPIx, &g_spi_driver_arch_data.SPI_InitStruct[dev->id]); 
			ret = 0;
			break;
		}
		case SPI_IOC_WR_LSB_FIRST:{
			ival = *((unsigned int*)arguments);
			g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_FirstBit = ival ? SPI_FirstBit_LSB : SPI_FirstBit_MSB;
			SPI_Init(SPIx, &g_spi_driver_arch_data.SPI_InitStruct[dev->id]); 
			ret = 0;
			break;
		}
		case SPI_IOC_WR_BITS_PER_WORD:{
			ival = *((unsigned int*)arguments);
			g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_DataSize = (ival == 16) ? SPI_DataSize_16b : SPI_DataSize_8b;
			SPI_Init(SPIx, &g_spi_driver_arch_data.SPI_InitStruct[dev->id]); 
			ret = 0;
			break;
		}
		case SPI_IOC_WR_MAX_SPEED_HZ:{
			ival = *((unsigned int*)arguments);
			scale_index = spi_drv_find_speed_near(ival, (dev->id == 0) ? 4 : 2);

			g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_BaudRatePrescaler = g_spi_driver_arch_data.speed_supported[scale_index].scale;
			SPI_Init(SPIx, &g_spi_driver_arch_data.SPI_InitStruct[dev->id]);
			ret = 0;
			break;
		}
		case SPI_IOC_RD_MAX_SPEED_HZ:{
			ival = spi_drv_find_speed_from_scale(g_spi_driver_arch_data.SPI_InitStruct[dev->id].SPI_BaudRatePrescaler, (dev->id == 0) ? 4 : 2);
			*((unsigned int*)arguments) = ival;
			ret = 0;
			break;
		}
		default:
			break;
	}
	return ret;
}
void SPI1_IRQHandler(void){
	static BaseType_t xHigherPriorityTaskWoken;
	static SPI_TypeDef* SPIx = SPI1;
	static uint8_t data;
	
	if( SPIx->SR & 0x01 ){
		SPIx->SR = 0x00;
		data = SPIx->DR;
		xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(g_spi_driver_arch_data.rx_event[0], &data, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}
void SPI2_IRQHandler(void){
	static BaseType_t xHigherPriorityTaskWoken;
	static SPI_TypeDef* SPIx = SPI2;
	static uint8_t data;
	
	if( SPIx->SR & 0x01 ){
		SPIx->SR = 0x00;
		data = SPIx->DR;
		xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(g_spi_driver_arch_data.rx_event[1], &data, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}
void SPI3_IRQHandler(void){
	static BaseType_t xHigherPriorityTaskWoken;
	static SPI_TypeDef* SPIx = SPI3;
	static uint8_t data;
	
	if( SPIx->SR & 0x01 ){
		SPIx->SR = 0x00;
		data = SPIx->DR;
		xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(g_spi_driver_arch_data.rx_event[2], &data, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}
#endif
// end of file
