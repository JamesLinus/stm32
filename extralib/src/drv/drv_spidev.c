#if defined(OS_UCOS)
#include <drv_api.h>
#include <drv_gpio.h>
#include <spidev.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_spi.h>
#include <stm32f4xx_hal_gpio.h>

#include <os.h>
#include <ringbuffer.h>
#include <bsp.h>
#include <debug.h>

int 	spi_init		(void);
int 	spi_open		(struct platform_device *dev, int flags);
int 	spi_close		(struct platform_device *dev);
int		spi_ioctl		(struct platform_device *dev, int request, unsigned int arguments);

void SPI1_IRQHandler(void);
void SPI2_IRQHandler(void);
void SPI3_IRQHandler(void);
#define SPI_DRIVER_SPEED_CNT	3
struct spi_driver_speed_ref{
	unsigned int speed;
	unsigned char scale;
};
#define SPI_DRIVER_RX_BUF_SIZE	(32)
struct spi_driver_arch_data{
	OS_FLAG_GRP						rx_event;
	struct RingBuffer				rx_buf[SPI_DRIVER_SPEED_CNT];
	uint8_t							__rx_buf[SPI_DRIVER_SPEED_CNT][SPI_DRIVER_RX_BUF_SIZE];
	struct spi_driver_speed_ref 	speed_supported[SPI_DRIVER_SPEED_CNT];
	SPI_HandleTypeDef				handle[SPI_DRIVER_SPEED_CNT];
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
	OS_ERR err;
	memset(&g_spi_driver_arch_data, 0, sizeof(g_spi_driver_arch_data));
	g_spi_driver_arch_data.speed_supported[0].scale = SPI_BAUDRATEPRESCALER_2;
	g_spi_driver_arch_data.speed_supported[0].speed = SystemCoreClock / 2;
	g_spi_driver_arch_data.speed_supported[1].scale = SPI_BAUDRATEPRESCALER_4;
	g_spi_driver_arch_data.speed_supported[1].speed = SystemCoreClock / 4;
	g_spi_driver_arch_data.speed_supported[2].scale = SPI_BAUDRATEPRESCALER_8;
	g_spi_driver_arch_data.speed_supported[2].speed = SystemCoreClock / 8;
	g_spi_driver_arch_data.speed_supported[3].scale = SPI_BAUDRATEPRESCALER_16;
	g_spi_driver_arch_data.speed_supported[3].speed = SystemCoreClock / 16;
	g_spi_driver_arch_data.speed_supported[4].scale = SPI_BAUDRATEPRESCALER_32;
	g_spi_driver_arch_data.speed_supported[4].speed = SystemCoreClock / 32;
	g_spi_driver_arch_data.speed_supported[5].scale = SPI_BAUDRATEPRESCALER_64;
	g_spi_driver_arch_data.speed_supported[5].speed = SystemCoreClock / 64;
	g_spi_driver_arch_data.speed_supported[6].scale = SPI_BAUDRATEPRESCALER_128;
	g_spi_driver_arch_data.speed_supported[6].speed = SystemCoreClock / 128;
	g_spi_driver_arch_data.speed_supported[7].scale = SPI_BAUDRATEPRESCALER_256;
	g_spi_driver_arch_data.speed_supported[7].speed = SystemCoreClock / 256;

	for(i = 0 ;i < SPI_DRIVER_SPEED_CNT; i++){
		RingBuffer_Init(&g_spi_driver_arch_data.rx_buf[i], SPI_DRIVER_RX_BUF_SIZE ,&g_spi_driver_arch_data.__rx_buf[i][0]);
	}
	OSFlagCreate(&g_spi_driver_arch_data.rx_event, "", 0, &err);

	platform_driver_register(&g_spi_driver);
	return 0;
}
module_init(spi_init);
int 	spi_open	(struct platform_device *dev, int flags){
	int ret;
	struct spi_platform_data* data;	
	GPIO_InitTypeDef  GPIO_InitStruct;
	IRQn_Type iRQn_Type;

	int bank, pin;
	
	ret = -EPERM;
	data = (struct spi_platform_data*)dev->dev.platform_data;
	
	if(dev->id < 0 || dev->id > SPI_MODULE_COUNT) return ret;
	
	switch(dev->id){
		case 0:{
			__HAL_RCC_SPI1_CLK_ENABLE();
			GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
			iRQn_Type = SPI1_IRQn;
			g_spi_driver_arch_data.handle[dev->id].Instance = SPI1;
			BSP_IntVectSet(BSP_INT_ID_SPI1, &SPI1_IRQHandler);
			break;
		}
		case 1:{
			__HAL_RCC_SPI2_CLK_ENABLE();
			GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
			iRQn_Type = SPI2_IRQn;
			g_spi_driver_arch_data.handle[dev->id].Instance = SPI2;
			BSP_IntVectSet(BSP_INT_ID_SPI2, &SPI2_IRQHandler);
			break;
		}
		case 2:{
			__HAL_RCC_SPI3_CLK_ENABLE();
			GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
			iRQn_Type = SPI3_IRQn;
			g_spi_driver_arch_data.handle[dev->id].Instance = SPI3;
			BSP_IntVectSet(BSP_INT_ID_SPI3, &SPI3_IRQHandler);
			break;
		}
		default:
			break;
	}
	
	// GPIO pin
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	if(data->sck_pin != GPIO_PIN_INVALID){
		bank = gpio_get_bank_index(data->sck_pin);
		pin = gpio_get_pin_index(data->sck_pin);
		switch(bank){
			case 0: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
			case 1: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
			case 2: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
			case 3: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
			case 4: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
			case 5: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
			case 6: __HAL_RCC_GPIOG_CLK_ENABLE(); break;
			case 7: __HAL_RCC_GPIOH_CLK_ENABLE(); break;
			case 8: __HAL_RCC_GPIOI_CLK_ENABLE(); break;
			default: break;
		}
		GPIO_InitStruct.Pin       = g_gpio_pin_ref[pin].GPIO_Pin;
		HAL_GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIO_InitStruct);
	}
	if(data->mosi_pin != GPIO_PIN_INVALID){
		bank = gpio_get_bank_index(data->mosi_pin);
		pin = gpio_get_pin_index(data->mosi_pin);		
		switch(bank){
			case 0: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
			case 1: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
			case 2: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
			case 3: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
			case 4: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
			case 5: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
			case 6: __HAL_RCC_GPIOG_CLK_ENABLE(); break;
			case 7: __HAL_RCC_GPIOH_CLK_ENABLE(); break;
			case 8: __HAL_RCC_GPIOI_CLK_ENABLE(); break;
			default: break;
		}
		GPIO_InitStruct.Pin       = g_gpio_pin_ref[pin].GPIO_Pin;
		HAL_GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIO_InitStruct);
	}
	if(data->miso_pin != GPIO_PIN_INVALID){
		bank = gpio_get_bank_index(data->miso_pin);
		pin = gpio_get_pin_index(data->miso_pin);
		switch(bank){
			case 0: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
			case 1: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
			case 2: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
			case 3: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
			case 4: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
			case 5: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
			case 6: __HAL_RCC_GPIOG_CLK_ENABLE(); break;
			case 7: __HAL_RCC_GPIOH_CLK_ENABLE(); break;
			case 8: __HAL_RCC_GPIOI_CLK_ENABLE(); break;
			default: break;
		}
		GPIO_InitStruct.Pin       = g_gpio_pin_ref[pin].GPIO_Pin;
		HAL_GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIO_InitStruct);
	}
	if(data->ss_pin != GPIO_PIN_INVALID){
		bank = gpio_get_bank_index(data->ss_pin);
		pin = gpio_get_pin_index(data->ss_pin);		
		switch(bank){
			case 0: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
			case 1: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
			case 2: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
			case 3: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
			case 4: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
			case 5: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
			case 6: __HAL_RCC_GPIOG_CLK_ENABLE(); break;
			case 7: __HAL_RCC_GPIOH_CLK_ENABLE(); break;
			case 8: __HAL_RCC_GPIOI_CLK_ENABLE(); break;
			default: break;
		}
		GPIO_InitStruct.Pin       = g_gpio_pin_ref[pin].GPIO_Pin;
		HAL_GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIO_InitStruct);
		
//		SPI_SSOutputCmd(SPIx, ENABLE);
	}
	/* configure SPI in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	g_spi_driver_arch_data.handle[dev->id].Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	g_spi_driver_arch_data.handle[dev->id].Init.Direction         = SPI_DIRECTION_2LINES;
	g_spi_driver_arch_data.handle[dev->id].Init.CLKPhase          = SPI_PHASE_1EDGE;
	g_spi_driver_arch_data.handle[dev->id].Init.CLKPolarity       = SPI_POLARITY_LOW;
	g_spi_driver_arch_data.handle[dev->id].Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	g_spi_driver_arch_data.handle[dev->id].Init.CRCPolynomial     = 7;
	g_spi_driver_arch_data.handle[dev->id].Init.DataSize          = SPI_DATASIZE_8BIT;
	g_spi_driver_arch_data.handle[dev->id].Init.FirstBit          = SPI_FIRSTBIT_MSB;
	g_spi_driver_arch_data.handle[dev->id].Init.NSS               = SPI_NSS_SOFT;
	g_spi_driver_arch_data.handle[dev->id].Init.TIMode            = SPI_TIMODE_DISABLE;
	g_spi_driver_arch_data.handle[dev->id].Init.Mode 			  = SPI_MODE_MASTER;
	HAL_SPI_Init(&g_spi_driver_arch_data.handle[dev->id]);
	// interrupt
	HAL_NVIC_SetPriority(iRQn_Type, 7, 0);
	HAL_NVIC_EnableIRQ(iRQn_Type);
	__HAL_SPI_ENABLE_IT(&g_spi_driver_arch_data.handle[dev->id], SPI_IT_RXNE);
	__HAL_SPI_ENABLE(&g_spi_driver_arch_data.handle[dev->id]);
	
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
	struct spi_ioc_transfer	    *tr;
	unsigned char *tx, *rx, u8val;
	unsigned int  len;
	unsigned int ival;
	uint16_t sr;
	uint16_t scale_index;
	OS_FLAGS flags;
	OS_ERR err;
	
	ret = -EPERM;
	data = (struct spi_platform_data*)dev->dev.platform_data;
	
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
				g_spi_driver_arch_data.handle[dev->id].Init.BaudRatePrescaler = g_spi_driver_arch_data.speed_supported[scale_index].scale;
				ival = 1;
			}
			if(tr->bits_per_word > 0){
				g_spi_driver_arch_data.handle[dev->id].Init.DataSize = (tr->bits_per_word == 16) ? SPI_DATASIZE_16BIT : SPI_DATASIZE_8BIT;
				ival = 1;
			}
			if(ival){
				HAL_SPI_Init(&g_spi_driver_arch_data.handle[dev->id]);
				__HAL_SPI_ENABLE_IT(&g_spi_driver_arch_data.handle[dev->id], SPI_IT_RXNE);
				__HAL_SPI_ENABLE(&g_spi_driver_arch_data.handle[dev->id]);
			}
			while(len > 0){
				if(tx){
					g_spi_driver_arch_data.handle[dev->id].Instance->DR = *tx;
					tx++;
				}
				else
					g_spi_driver_arch_data.handle[dev->id].Instance->DR = 0;

				flags = OSFlagPend(&g_spi_driver_arch_data.rx_event,
						((OS_FLAGS)1) << dev->id,
						TICK_RATE_HZ/100,
						OS_OPT_PEND_FLAG_SET_ANY|OS_OPT_PEND_FLAG_CONSUME,
						0,
						&err);
				if(flags & (((OS_FLAGS)1) << dev->id)){
					RingBuffer_Pop(&g_spi_driver_arch_data.rx_buf[dev->id], &u8val, 1);
				}
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
				g_spi_driver_arch_data.handle[dev->id].Init.CLKPolarity 	= SPI_POLARITY_LOW;
				g_spi_driver_arch_data.handle[dev->id].Init.CLKPhase 		= SPI_PHASE_1EDGE;

			}
			if(ival & 0x01){
				g_spi_driver_arch_data.handle[dev->id].Init.CLKPolarity 	= SPI_POLARITY_HIGH;
			}
			if(ival & 0x02){
				g_spi_driver_arch_data.handle[dev->id].Init.CLKPhase 		= SPI_PHASE_2EDGE;
			}
			if(ival & SPI_NO_CS){
//				SPI_SSOutputCmd(SPIx, DISABLE);
			}
			if(ival & SPI_LSB_FIRST){
				g_spi_driver_arch_data.handle[dev->id].Init.FirstBit          = SPI_FIRSTBIT_LSB;
			}else{
				g_spi_driver_arch_data.handle[dev->id].Init.FirstBit          = SPI_FIRSTBIT_MSB;
			}
			HAL_SPI_Init(&g_spi_driver_arch_data.handle[dev->id]);
			__HAL_SPI_ENABLE_IT(&g_spi_driver_arch_data.handle[dev->id], SPI_IT_RXNE);
			__HAL_SPI_ENABLE(&g_spi_driver_arch_data.handle[dev->id]);
			ret = 0;
			break;
		}
		case SPI_IOC_WR_LSB_FIRST:{
			ival = *((unsigned int*)arguments);
			g_spi_driver_arch_data.handle[dev->id].Init.FirstBit = ival ? SPI_FIRSTBIT_LSB : SPI_FIRSTBIT_MSB;
			HAL_SPI_Init(&g_spi_driver_arch_data.handle[dev->id]);
			__HAL_SPI_ENABLE_IT(&g_spi_driver_arch_data.handle[dev->id], SPI_IT_RXNE);
			__HAL_SPI_ENABLE(&g_spi_driver_arch_data.handle[dev->id]);
			ret = 0;
			break;
		}
		case SPI_IOC_WR_BITS_PER_WORD:{
			ival = *((unsigned int*)arguments);
			g_spi_driver_arch_data.handle[dev->id].Init.DataSize = (ival == 16) ? SPI_DATASIZE_16BIT : SPI_DATASIZE_8BIT;
			HAL_SPI_Init(&g_spi_driver_arch_data.handle[dev->id]);
			__HAL_SPI_ENABLE_IT(&g_spi_driver_arch_data.handle[dev->id], SPI_IT_RXNE);
			__HAL_SPI_ENABLE(&g_spi_driver_arch_data.handle[dev->id]);
			ret = 0;
			break;
		}
		case SPI_IOC_WR_MAX_SPEED_HZ:{
			ival = *((unsigned int*)arguments);
			scale_index = spi_drv_find_speed_near(ival, (dev->id == 0) ? 4 : 2);

			g_spi_driver_arch_data.handle[dev->id].Init.BaudRatePrescaler = g_spi_driver_arch_data.speed_supported[scale_index].scale;
			HAL_SPI_Init(&g_spi_driver_arch_data.handle[dev->id]);
			__HAL_SPI_ENABLE_IT(&g_spi_driver_arch_data.handle[dev->id], SPI_IT_RXNE);
			__HAL_SPI_ENABLE(&g_spi_driver_arch_data.handle[dev->id]);
			ret = 0;
			break;
		}
		case SPI_IOC_RD_MAX_SPEED_HZ:{
			ival = spi_drv_find_speed_from_scale(g_spi_driver_arch_data.handle[dev->id].Init.BaudRatePrescaler, (dev->id == 0) ? 4 : 2);
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
	static uint8_t data;
	static OS_ERR p_err;
	
	if( g_spi_driver_arch_data.handle[0].Instance->SR & 0x01 ){
		g_spi_driver_arch_data.handle[0].Instance->SR = 0x00;
		data = g_spi_driver_arch_data.handle[0].Instance->DR;
		RingBuffer_Push(&g_spi_driver_arch_data.rx_buf[0], &data, 1);
		OSFlagPost(&g_spi_driver_arch_data.rx_event, ((OS_FLAGS)1) << 0, OS_OPT_POST_FLAG_SET, &p_err);
	}
}
void SPI2_IRQHandler(void){
	static uint8_t data;
	static OS_ERR p_err;
	if( g_spi_driver_arch_data.handle[1].Instance->SR & 0x01 ){
		g_spi_driver_arch_data.handle[1].Instance->SR = 0x00;
		data = g_spi_driver_arch_data.handle[1].Instance->DR;
		RingBuffer_Push(&g_spi_driver_arch_data.rx_buf[1], &data, 1);
		OSFlagPost(&g_spi_driver_arch_data.rx_event, ((OS_FLAGS)1) << 1, OS_OPT_POST_FLAG_SET, &p_err);
	}
}
void SPI3_IRQHandler(void){
	static uint8_t data;
	static OS_ERR p_err;
	
	if( g_spi_driver_arch_data.handle[2].Instance->SR & 0x01 ){
		g_spi_driver_arch_data.handle[2].Instance->SR = 0x00;
		data = g_spi_driver_arch_data.handle[2].Instance->DR;
		RingBuffer_Push(&g_spi_driver_arch_data.rx_buf[2], &data, 1);
		OSFlagPost(&g_spi_driver_arch_data.rx_event, ((OS_FLAGS)1) << 2, OS_OPT_POST_FLAG_SET, &p_err);
	}
}
#endif
// end of file
