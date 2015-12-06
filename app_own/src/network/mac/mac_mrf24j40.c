/*
 * mac_mrf24j40.cpp
 *
 *  Created on: Nov 29, 2015
 *      Author: dev
 */

#include "mac_mrf24j40.h"
#include "mac_mrf24j40_defs.h"
#include <unistd.h>
#include <fcntl.h>
#include <spidev.h>
#include <drv_gpio.h>
#include <debug.h>

#define PHY_SET_REG_VERIFY	0

uint8_t PHY_mrf24j40_getLongRAMAddr	(struct phy_mrf24j40* phy,uint16_t address){
	uint8_t ret = 0;
	uint8_t u8tx[3], u8rx[3];
	struct spi_ioc_transfer xfer;

	u8tx[0] = 0;
	write(phy->fd_cs, u8tx, 1);	// active cs
	xfer.tx_buf			= (unsigned int)u8tx;
	xfer.rx_buf 		= (unsigned int)u8rx;
	xfer.bits_per_word 	= 0;
	xfer.speed_hz 		= 0;
	xfer.len			= 3;
	u8tx[0] = ((address >> 3)&0x7F) | 0x80;
	u8tx[1] = ((address << 5)&0xE0);
	u8tx[2] = 0;
	u8rx[2] = 0;
	ioctl(phy->fd_spi, SPI_IOC_MESSAGE(1), (unsigned int)&xfer);
	u8tx[0] = 1;
	write(phy->fd_cs, u8tx, 1);	// deactive cs
	return u8rx[2];
}
uint8_t PHY_mrf24j40_getShortRAMAddr(struct phy_mrf24j40* phy,uint8_t address){
	uint8_t ret = 0;
	uint8_t u8tx[2], u8rx[2];
	struct spi_ioc_transfer xfer;

	u8tx[0] = 0;
	write(phy->fd_cs, u8tx, 1);	// active cs
	xfer.tx_buf			= (unsigned int)u8tx;
	xfer.rx_buf 		= (unsigned int)u8rx;
	xfer.bits_per_word 	= 0;
	xfer.speed_hz 		= 0;
	xfer.len			= 2;
	u8tx[0] 			= address;
	u8tx[1]				= 0;
	u8rx[1] 			= 0;
	ioctl(phy->fd_spi, SPI_IOC_MESSAGE(1), (unsigned int)&xfer);
	u8tx[0] = 1;
	write(phy->fd_cs, u8tx, 1);	// deactive cs
	return u8rx[1];
}
void 	PHY_mrf24j40_setLongRAMAddr	(struct phy_mrf24j40* phy,uint16_t address, uint8_t value){
	uint8_t u8tx[3];
	struct spi_ioc_transfer xfer;

	u8tx[0] = 0;
	write(phy->fd_cs, u8tx, 1);	// active cs
	xfer.tx_buf			= (unsigned int)u8tx;
	xfer.rx_buf 		= 0;
	xfer.bits_per_word 	= 0;
	xfer.speed_hz 		= 0;
	xfer.len			= 3;
	u8tx[0] = (((address >> 3))&0x7F) | 0x80;
	u8tx[1] = (((address << 5))&0xE0) | 0x10;
	u8tx[2] = value;
	ioctl(phy->fd_spi, SPI_IOC_MESSAGE(1), (unsigned int)&xfer);
	u8tx[0] = 1;
	write(phy->fd_cs, u8tx, 1);	// deactive cs
#if (PHY_SET_REG_VERIFY > 0)
	u8tx[0] = PHY_mrf24j40_getLongRAMAddr(phy, address);
	if(u8tx[0] != value){
		LREP_WARN("PHY set reg %04X=%02X failed, read back %02X\r\n",
				address, value, u8tx[0]);
	}
#endif
}
void 	PHY_mrf24j40_setShortRAMAddr(struct phy_mrf24j40* phy,uint8_t address, uint8_t value){
	uint8_t u8tx[2];
	struct spi_ioc_transfer xfer;

	u8tx[0] = 0;
	write(phy->fd_cs, u8tx, 1);	// active cs
	xfer.tx_buf			= (unsigned int)u8tx;
	xfer.rx_buf 		= 0;
	xfer.bits_per_word 	= 0;
	xfer.speed_hz 		= 0;
	xfer.len			= 2;
	u8tx[0] = address;
	u8tx[1] = value;
	ioctl(phy->fd_spi, SPI_IOC_MESSAGE(1), (unsigned int)&xfer);
	u8tx[0] = 1;
	write(phy->fd_cs, u8tx, 1);	// deactive cs
	// verify
#if (PHY_SET_REG_VERIFY > 0)
	if(address != PHY_MRF24J40_WRITE_SOFTRST){
		u8tx[0] = PHY_mrf24j40_getShortRAMAddr(phy, address & (~((uint8_t)0x01)));
		if(address == PHY_MRF24J40_WRITE_RXFLUSH){
			u8tx[0] &= ((uint8_t)0x6E);
			value &= ~((uint8_t)0x01);
		}
		else if(address == PHY_MRF24J40_WRITE_BBREG6) u8tx[0] &= (uint8_t)(0x40);
		if(u8tx[0] != value){
			LREP_WARN("PHY set reg %02X=%02X failed, read back %02X\r\n",
					address, value, u8tx[0]);
		}
	}
#endif
}
void PHY_mrf24j40_hardreset(struct phy_mrf24j40* phy){
	uint8_t u8val = 0;

	write(phy->fd_reset, &u8val, 1);
	usleep_s(1000 * 100);
	u8val = 1;
	write(phy->fd_reset, &u8val, 1);
	usleep_s(1000 * 100);
}
void PHY_mrf24j40_softreset(struct phy_mrf24j40* phy){
	uint8_t i;
	uint8_t timeout = 100;
	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_SOFTRST, 0x07);
    do
    {
        i = PHY_mrf24j40_getShortRAMAddr(phy, PHY_MRF24J40_READ_SOFTRST);
        usleep_s(1000 * 10);
        timeout-=10;
    } while (((i & 0x07) != (uint8_t) 0x00) && (timeout > 0));
    usleep_s(1000 * 100);
    if(timeout == 0) LREP_WARN("timeout\r\n");
}
int PHY_mrf24j40_setChannel(struct phy_mrf24j40* phy, uint8_t channel)
{
    if (channel < 11 || channel > 26)
    {
        return -1;
    }

#if defined(ENABLE_PA_LNA) && (defined(MRF24J40MB) || defined(MRF24J40MC))
    if (channel == 26)
    {
        return false;
    }
#endif
    phy->channel = channel;
    PHY_mrf24j40_setLongRAMAddr(phy, PHY_MRF24J40_RFCTRL0, ((channel - 11) << 4) | 0x03);
    PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_RFCTL, 0x04);
    PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_RFCTL, 0x00);
    return 0;
}
void PHY_mrf24j40_initialize(struct phy_mrf24j40* phy){
	uint8_t i;
	uint8_t timeout = 100;
	PHY_mrf24j40_hardreset(phy);
	PHY_mrf24j40_softreset(phy);

   /* flush the RX fifo */
	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_RXFLUSH, 0x01);

	/* Program the short MAC Address, 0xffff */
	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_SADRL, 0xFF);
	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_SADRH, 0xFF);
	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_PANIDL, 0xFF);
	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_PANIDH, 0xFF);

	/* Program Long MAC Address*/
	for (i = 0; i < (uint8_t) 8; i++)
	{
		PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_EADR0 + i * 2, phy->l_address[i]);
	}

	/* setup */
	PHY_mrf24j40_setLongRAMAddr(phy, PHY_MRF24J40_RFCTRL2, 0x80);

#if defined(ENABLE_PA_LNA)
#if defined(MRF24J40MB)
	// There are special MRF24J40 transceiver output power
	// setting for Microchip MRF24J40MB module.
#if APPLICATION_SITE == EUROPE
	// MRF24J40 output power set to be -14.9dB
	PHY_mrf24j40_setLongRAMAddr(phy, PHY_MRF24J40_RFCTRL3, 0x70);
#else
	// MRF24J40 output power set to be -1.9dB
	PHY_mrf24j40_setLongRAMAddr(phy, PHY_MRF24J40_RFCTRL3, 0x18);
#endif
#elif defined(MRF24J40MC)
	// MRF24J40 output power set to be -3.7dB for MRF24J40MB
	PHY_mrf24j40_setLongRAMAddr(phy, PHY_MRF24J40_RFCTRL3, 0x28);
#else
	// power level set to be 0dBm, must adjust according to
	// FCC/IC/ETSI requirement
	PHY_mrf24j40_setLongRAMAddr(phy, PHY_MRF24J40_RFCTRL3, 0x00);
#endif
#else
	// power level to be 0dBm
	PHY_mrf24j40_setLongRAMAddr(phy, PHY_MRF24J40_RFCTRL3, 0x00);
#endif

	/* program RSSI ADC with 2.5 MHz clock */
	PHY_mrf24j40_setLongRAMAddr(phy, PHY_MRF24J40_RFCTRL6, 0x90);

	PHY_mrf24j40_setLongRAMAddr(phy, PHY_MRF24J40_RFCTRL7, 0x80);

	PHY_mrf24j40_setLongRAMAddr(phy, PHY_MRF24J40_RFCTRL8, 0x10);

	PHY_mrf24j40_setLongRAMAddr(phy, PHY_MRF24J40_SCLKDIV, 0x21);

	/* Program CCA mode using RSSI */
	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_BBREG2, 0x80);
	/* Enable the packet RSSI */
	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_BBREG6, 0x40);
	/* Program CCA, RSSI threshold values */
	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_RSSITHCCA, 0x60);

#if defined(ENABLE_PA_LNA)

#if defined(MRF24J40MC)
	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_GPIODIR, 0x08);
	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_GPIO, 0x08);
#endif
	PHY_mrf24j40_setLongRAMAddr(phy, PHY_MRF24J40_TESTMODE, 0x0F);

#endif
	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_FFOEN, 0x98);
	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_TXPEMISP, 0x95);

	// wait until the MRF24J40 in receive mode
	do
	{
		i = PHY_mrf24j40_getLongRAMAddr(phy, PHY_MRF24J40_RFSTATE);
		usleep_s(1000);
		timeout--;
	}
	while (((i&0xA0) != 0xA0) && (timeout > 0));
	if(timeout == 0) LREP_WARN("timeout\r\n");

	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_INTMSK, 0xE6);

#ifdef ENABLE_INDIRECT_MESSAGE
	PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_ACKTMOUT, 0xB9);
#endif

	// Make RF communication stable under extreme temperatures
	PHY_mrf24j40_setLongRAMAddr(phy, PHY_MRF24J40_RFCTRL0, 0x03);
	PHY_mrf24j40_setLongRAMAddr(phy, PHY_MRF24J40_RFCTRL1, 0x02);
	PHY_mrf24j40_setChannel(phy, phy->channel);
	// Define TURBO_MODE if more bandwidth is required
	// to enable radio to operate to TX/RX maximum
	// 625Kbps
	if(phy->turboEn){
		PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_BBREG0, 0x01);
		PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_BBREG3, 0x38);
		PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_BBREG4, 0x5C);

		PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_RFCTL, 0x04);
		PHY_mrf24j40_setShortRAMAddr(phy, PHY_MRF24J40_WRITE_RFCTL, 0x00);
	}
}
#define __mac_mrf24j40_lock(mac) sem_wait(&mac->sem_access)
#define __mac_mrf24j40_unlock(mac) sem_post(&mac->sem_access)
int 	MAC_mrf24j40_open(struct mac_mrf24j40* mac, struct mac_mrf24j40_open_param *init){
	int ret = -1, i;

	mac->phy.fd_spi 	= init->fd_spi;
	mac->phy.fd_cs	 	= init->fd_cs;
	mac->phy.fd_reset	= init->fd_reset;
	mac->phy.fd_intr	= init->fd_intr;
	mac->phy.l_address[0] = 0;
	mac->phy.l_address[1] = 0;
	mac->phy.l_address[2] = 0;
	mac->phy.l_address[3] = 0;
	mac->phy.l_address[4] = 0;
	mac->phy.l_address[5] = 0;
	mac->phy.l_address[6] = 0;
	mac->phy.l_address[7] = 0;
	mac->phy.channel	  = 25;
	mac->phy.turboEn 	  = 0;

	mac->txSeq = 0;
	mac->networkAddress = 0;
	mac->flags = 0x01;

	for(i = 0; i < MAC_MRF24J40_WRITE_MAX_ITEMS; i++){
		mac->write_items[i].flags = 0x00;
	}
	for(i = 0; i < MAC_MRF24J40_READ_MAX_ITEMS; i++){
		mac->read_items[i].flags = 0x00;
	}

	flag_event_init(&mac->event);
	flag_event_init(&mac->rx_event);
	sem_init(&mac->sem_access, 0, 1);

	PHY_mrf24j40_initialize(&mac->phy);
	ret = 0;

	return ret;
}
int 	MAC_mrf24j40_read(struct mac_mrf24j40* mac, void* payload, int payload_maxlen, int timeout){
	int ret = 0, i;
	struct mac_mrf24j40_read_item* item = 0;
	struct timespec abs_timeout;

	__mac_mrf24j40_lock(mac);
	for(i = 0; i < MAC_MRF24J40_READ_MAX_ITEMS; i++){
		if(mac->read_items[i].flags & 0x01){
			item = &mac->read_items[i];
			break;
		}
	}
	if(item){
		if(item->payload_len > payload_maxlen) item->payload_len = payload_maxlen;
		_lib_memcpy(payload, item->payload, item->payload_len);
		ret = item->payload_len;
		item->flags &= ~((uint8_t)0x01);
	}
	__mac_mrf24j40_unlock(mac);
	if(!item && timeout > 0){
		abs_timeout.tv_sec = timeout / 1000;
		abs_timeout.tv_nsec= (timeout % 1000) * 1000000;
		flag_event_timedwait(&mac->rx_event, &abs_timeout);
		__mac_mrf24j40_lock(mac);
		for(i = 0; i < MAC_MRF24J40_READ_MAX_ITEMS; i++){
			if(mac->read_items[i].flags & 0x01){
				item = &mac->read_items[i];
				break;
			}
		}
		if(item){
			if(item->payload_len > payload_maxlen) item->payload_len = payload_maxlen;
			_lib_memcpy(payload, item->payload, item->payload_len);
			ret = item->payload_len;
			item->flags &= ~((uint8_t)0x01);
		}
		__mac_mrf24j40_unlock(mac);
	}
	return ret;
}
int 	MAC_mrf24j40_write(struct mac_mrf24j40* mac, struct mac_mrf24j40_write_param* trans,
		void* payload, int payloadlen){
	int ret = 0, i;
	struct mac_mrf24j40_write_item* item = 0;
	__mac_mrf24j40_lock(mac);
	// find free item
	for(i = 0 ;i < MAC_MRF24J40_WRITE_MAX_ITEMS; i++){
		if((mac->write_items[i].flags & 0x01) == 0){
			item = &mac->write_items[i];
			break;
		}
	}
	if(item){
		if(payloadlen > MAC_MRF24J40_WRITE_PAYLOAD_MAX_LENGTH) payloadlen = MAC_MRF24J40_WRITE_PAYLOAD_MAX_LENGTH;
		_lib_memcpy(&item->param, trans, sizeof(struct mac_mrf24j40_write_param));
		_lib_memcpy(item->payload, payload, payloadlen);
		item->payload_len = payloadlen;
		item->flags |= 0x01;
	}else{
		LREP_WARN("no more tx space\r\n");
		ret = -1;
	}
	__mac_mrf24j40_unlock(mac);
	flag_event_post(&mac->event);

	return 0;
}
int 	__mac_mrf24j40_write(struct mac_mrf24j40* mac, struct mac_mrf24j40_write_param* trans,
		void* payload, int payloadlen){
	int ret = -1, i;
	uint8_t hdr_len;
	uint16_t ram_addr, ram_addr_tmp;
	struct mac_ieee802154_frm_ctrl frmCtrl;
	uint8_t *p;
	/*
	 * pkt: | hdr_len(1) | hdr_len + payload_len | payload |
	 * payload: | frmCtrl(2) | seq(1) | destPANId(2) | destAddr(2/8) | srcPANId(0/2) | srcAddr(2/8) | payload | frmCheckSeq |
	 * srcPANId is omitted if frmCtrl.intraPAN = 1
	 * destAddr = 0xFFFF if broadcast
	 * */

	frmCtrl.bits.Val = 0;

	if(trans->flags.bits.packetType == MAC_MRF24J40_PACKET_TYPE_DATA)
		frmCtrl.bits.frmType = mac_iee802154_frmtype_data;
	else if(trans->flags.bits.packetType == MAC_MRF24J40_PACKET_TYPE_CMD)
		frmCtrl.bits.frmType = mac_iee802154_frmtype_cmd;
	// inner PAN
	hdr_len = 5;
	frmCtrl.bits.intraPAN = 1;
	//altDestAddr = 1
	hdr_len += 2;
	//altSrcAddr = 1
	hdr_len += 2;
	// ACK req
	if(trans->flags.bits.ackReq && (trans->flags.bits.broadcast == 0))
		frmCtrl.bits.ackReq = 1;
	frmCtrl.bits.destAddrMode = mac_iee802154_addrmode_16bit;
	frmCtrl.bits.srcAddrMode = mac_iee802154_addrmode_16bit;

	ram_addr = 0;
	PHY_mrf24j40_setLongRAMAddr(&mac->phy, ram_addr++, hdr_len);
	PHY_mrf24j40_setLongRAMAddr(&mac->phy, ram_addr++, hdr_len + payloadlen);
	PHY_mrf24j40_setLongRAMAddr(&mac->phy, ram_addr++, (frmCtrl.bits.Val & 0x00FF));
	PHY_mrf24j40_setLongRAMAddr(&mac->phy, ram_addr++, ((frmCtrl.bits.Val & 0xFF00) >> 8));
	PHY_mrf24j40_setLongRAMAddr(&mac->phy, ram_addr++, mac->txSeq++);
	PHY_mrf24j40_setLongRAMAddr(&mac->phy, ram_addr++, (trans->destPANId & 0x00FF));
	PHY_mrf24j40_setLongRAMAddr(&mac->phy, ram_addr++, ((trans->destPANId & 0xFF00) >> 8));
	if(trans->flags.bits.broadcast){
		PHY_mrf24j40_setLongRAMAddr(&mac->phy, ram_addr++, 0xFF);
		PHY_mrf24j40_setLongRAMAddr(&mac->phy, ram_addr++, 0xFF);
	}else{
		PHY_mrf24j40_setLongRAMAddr(&mac->phy, ram_addr++, trans->destAddress & 0x00FF);
		PHY_mrf24j40_setLongRAMAddr(&mac->phy, ram_addr++, ((trans->destAddress & 0x00FF00) >> 8));
	}
	PHY_mrf24j40_setLongRAMAddr(&mac->phy, ram_addr++, mac->networkAddress & 0x00FF);
	PHY_mrf24j40_setLongRAMAddr(&mac->phy, ram_addr++, ((mac->networkAddress & 0x00FF00) >> 8));
	p = (uint8_t*)payload;
	for(i = 0; i < payloadlen; i++){
		PHY_mrf24j40_setLongRAMAddr(&mac->phy, ram_addr++, p[i]);
	}
#if MAC_PRINT_TX > 0
	ram_addr_tmp = 0;
	LREP("\r\n*** TX BEGIN ***\r\n");
	while(ram_addr_tmp < ram_addr){
		if((ram_addr_tmp % 16) == 0) LREP("\r\n");
		LREP("%02X ", PHY_mrf24j40_getLongRAMAddr(&mac->phy, ram_addr_tmp));
		ram_addr_tmp++;
	}
	LREP("\r\nTX DONE\r\n");
#endif
	// trigger tx
	if(frmCtrl.bits.ackReq == 1) i = 0x05;
	else i = 0x01;
	PHY_mrf24j40_setShortRAMAddr(&mac->phy, PHY_MRF24J40_WRITE_TXNMTRIG, i);

	ret = 0;
	return ret;
}
int 	MAC_mrf24j40_select(struct mac_mrf24j40* mac, int timeout){
	int ret = -1;
	struct timespec abs_timeout;

	abs_timeout.tv_sec = timeout / 1000;
	abs_timeout.tv_nsec= (timeout % 1000) * 1000000;
	ret = flag_event_timedwait(&mac->event, &abs_timeout);
	return ret;
}
int		MAC_mrf24j40_ioctl(struct mac_mrf24j40* mac, int request, unsigned int arguments){
	int ret = -1, i;
	unsigned int* puiVal;
	struct mac_channel_assessment *ch_assessment;

	switch(request){
		case mac_mrf24j40_ioc_trigger_interrupt:{
			__mac_mrf24j40_lock(mac);
			flag_event_post(&mac->event);
			__mac_mrf24j40_unlock(mac);
			ret = 0;
			break;
		}
		case mac_mrf24j40_ioc_set_channel:{
			puiVal = (unsigned int*)arguments;
			PHY_mrf24j40_setChannel(&mac->phy, *puiVal);
			ret = 0;
			break;
		}
		case mac_mrf24j40_ioc_channel_assessment:{
			ch_assessment = (struct mac_channel_assessment*)arguments;
			ret =1000;
			PHY_mrf24j40_setShortRAMAddr(&mac->phy, PHY_MRF24J40_WRITE_BBREG6, 0x80);
			ch_assessment->noise_level = PHY_mrf24j40_getShortRAMAddr(&mac->phy, PHY_MRF24J40_READ_BBREG6);
			while(((ch_assessment->noise_level & 0x01) != 0x01) && (ret > 0)){
				ch_assessment->noise_level = PHY_mrf24j40_getShortRAMAddr(&mac->phy, PHY_MRF24J40_READ_BBREG6);
				usleep_s(1000*10);
				ret -= 10;
			}
			ch_assessment->noise_level = PHY_mrf24j40_getLongRAMAddr(&mac->phy, 0x210);
			PHY_mrf24j40_setShortRAMAddr(&mac->phy, PHY_MRF24J40_WRITE_BBREG6, 0x40);
			if(ret > 0) ret = -1;
			else ret = 0;
			break;
		}
		case mac_mrf24j40_ioc_reset:{
			__mac_mrf24j40_lock(mac);
			mac->flags = 0x01;
			for(i = 0; i < MAC_MRF24J40_WRITE_MAX_ITEMS; i++){
				mac->write_items[i].flags = 0x00;
			}
			for(i = 0; i < MAC_MRF24J40_READ_MAX_ITEMS; i++){
				mac->read_items[i].flags = 0x00;
			}
			__mac_mrf24j40_unlock(mac);
			break;
		}
	}
	return ret;
}
int 	MAC_mrf24j40_task(struct mac_mrf24j40* mac){
	int ret = -1, i;
	uint8_t u8val, u8len;
	PHY_MRF24J40_IFREG	flags;
	uint8_t rxBuf[144];
	struct mac_mrf24j40_write_item* item = 0;
	struct mac_mrf24j40_read_item*	read_item = 0;

	// interrupt
	flags.Val = PHY_mrf24j40_getShortRAMAddr(&mac->phy, PHY_MRF24J40_READ_ISRSTS);
	if(flags.bits.RF_TXIF){
		mac->flags |= ((uint8_t)1 << MAC_MRF24J40_FLAG_TX_DONE);	// set bit tx done
		u8val = PHY_mrf24j40_getShortRAMAddr(&mac->phy, PHY_MRF24J40_READ_TXSR);
	}
	if(flags.bits.RF_RXIF){
		PHY_mrf24j40_setShortRAMAddr(&mac->phy, PHY_MRF24J40_WRITE_BBREG1, 0x40);	// Disable RX
		u8len = PHY_mrf24j40_getLongRAMAddr(&mac->phy, 0x300) + 2;
		if(u8len > 144) u8len = 144;
		rxBuf[0] = u8len;
		for(i = 1 ;i < u8len; i++){
			rxBuf[i] = PHY_mrf24j40_getLongRAMAddr(&mac->phy, 0x300 + i);
		}
		PHY_mrf24j40_setShortRAMAddr(&mac->phy, PHY_MRF24J40_WRITE_RXFLUSH, 0x01);
		PHY_mrf24j40_setShortRAMAddr(&mac->phy, PHY_MRF24J40_WRITE_BBREG1, 0x00);	// Enable RX
		// find free item
		__mac_mrf24j40_lock(mac);
		for(i = 0; i < MAC_MRF24J40_READ_MAX_ITEMS; i++){
			if((mac->read_items[i].flags & 0x01) == 0){
				read_item = &mac->read_items[i];
				break;
			}
		}
		if(read_item){
			if(u8len > MAC_MRF24J40_READ_PAYLOAD_MAX_LENGTH-1+10) u8len = MAC_MRF24J40_READ_PAYLOAD_MAX_LENGTH-1+10;
			_lib_memcpy(read_item->payload, &rxBuf[10], u8len + 1 - 10);
			read_item->payload_len = u8len+1-10;
			read_item->flags |= 0x01;	// set flags
			mac->flags |= ((uint8_t)1 << MAC_MRF24J40_FLAG_RX_DONE);
			flag_event_post(&mac->rx_event);
		}else{
			LREP_WARN("no more rx space\r\n");
		}
		__mac_mrf24j40_unlock(mac);
	}
	if(flags.bits.SECIF){
		PHY_mrf24j40_setShortRAMAddr(&mac->phy, PHY_MRF24J40_WRITE_SECCR0, 0x80);
		LREP("\r\nSECIF\r\n");
	}
	// tx pennding items
	if(mac->flags & ((uint8_t)1 << MAC_MRF24J40_FLAG_TX_DONE)){	// if tx done
		__mac_mrf24j40_lock(mac);
		// find full item
		for(i = 0 ;i < MAC_MRF24J40_WRITE_MAX_ITEMS; i++){
			if(mac->write_items[i].flags & 0x01){
				item = &mac->write_items[i];
				__mac_mrf24j40_write(mac, &item->param, item->payload, item->payload_len);
				item->flags &= ~((uint8_t)0x01);	// clear full bit
				mac->flags &= ~((uint8_t)1 << MAC_MRF24J40_FLAG_TX_DONE);	// clear tx done
				break;
			}
		}
		__mac_mrf24j40_unlock(mac);
	}

	return ret;
}
// end of file
