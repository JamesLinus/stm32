/*
 * mac_mrf24j40.h
 *
 *  Created on: Nov 29, 2015
 *      Author: dev
 */

#ifndef SRC_NETWORK_MAC_MAC_MRF24J40_H_
#define SRC_NETWORK_MAC_MAC_MRF24J40_H_

#include <stddef.h>
#include <stdint.h>
#include <flag_event.h>
#include <semaphore.h>

//#define MAC_PRINT_TX	1
//#define MAC_PRINT_RX	1

struct __attribute__((packed)) mac_mrf24j40_open_param{
	int fd_spi;
	int fd_cs;
	int fd_reset;
	int fd_intr;
};
struct __attribute__((packed)) mac_mrf24j40_write_param {
	uint64_t destAddress;
	uint16_t destPANId;
	union{
		uint8_t Val;
		struct{
			uint8_t	packetType 	: 2;// DATA,CMD, ACK,RESERVE
			uint8_t broadcast	: 1;
			uint8_t secEn		: 1;
			uint8_t repeat		: 1;// allow repeaters to forward the msg
			uint8_t ackReq		: 1;
			uint8_t destPrsnt	: 1;// dest address in the msg
			uint8_t sourcePrsnt : 1;// src address in the msg
		}bits;
	}flags;
	uint8_t  altDestAddr;	// alternative network adress as destination
	uint8_t  altSrcAddr;	// alternative network adress as source
};
struct __attribute__((packed)) phy_mrf24j40{
	int fd_spi;
	int fd_cs;
	int fd_reset;
	int fd_intr;
	uint8_t	l_address[8];
	uint8_t	channel;
	uint8_t turboEn;
};
#define MAC_MRF24J40_WRITE_PAYLOAD_MAX_LENGTH	(100)
#define MAC_MRF24J40_WRITE_MAX_ITEMS			(3)
#define MAC_MRF24J40_READ_PAYLOAD_MAX_LENGTH	(100)
#define MAC_MRF24J40_READ_MAX_ITEMS				(3)
struct __attribute__((packed)) mac_mrf24j40_write_item{
	struct mac_mrf24j40_write_param param;
	uint8_t flags;
	uint8_t payload_len;
	uint8_t	payload[MAC_MRF24J40_WRITE_PAYLOAD_MAX_LENGTH];
};
struct __attribute__((packed)) mac_mrf24j40_read_item{
	uint8_t flags;
	uint8_t payload_len;
	uint8_t	payload[MAC_MRF24J40_READ_PAYLOAD_MAX_LENGTH];
};
#define MAC_MRF24J40_FLAG_TX_DONE	0
#define MAC_MRF24J40_FLAG_RX_DONE	1
struct __attribute__((packed)) mac_mrf24j40{
	uint64_t		networkAddress;
	struct phy_mrf24j40 phy;
	uint8_t 		txSeq;
	flag_event_t	event;
	flag_event_t	rx_event;
	sem_t			sem_access;
	uint8_t			flags;
	struct mac_mrf24j40_write_item	write_items[MAC_MRF24J40_WRITE_MAX_ITEMS];
	struct mac_mrf24j40_read_item	read_items[MAC_MRF24J40_READ_MAX_ITEMS];
};
enum MAC_MRF24J40_PACKET_TYPE{
	MAC_MRF24J40_PACKET_TYPE_DATA = 0,
	MAC_MRF24J40_PACKET_TYPE_CMD,
};

struct __attribute__((packed)) mac_ieee802154_frm_ctrl{
	union{
		uint16_t	Val;
		struct{
			uint16_t frmType	: 3;
			uint16_t secEn		: 1;
			uint16_t frmPending	: 1;
			uint16_t ackReq		: 1;
			uint16_t intraPAN	: 1;		// msg in current PAN
			uint16_t reserver1   : 3;
			uint16_t destAddrMode: 2;
			uint16_t reserver2   : 2;
			uint16_t srcAddrMode : 2;
		};
	}bits;
};
enum mac_iee802154_frmtype{
	mac_iee802154_frmtype_data = 0x01,
	mac_iee802154_frmtype_ack,
	mac_iee802154_frmtype_cmd
};
enum mac_iee802154_addrmode{
	mac_iee802154_addrmode_16bit = 0x02,
	mac_iee802154_addrmode_32bit = 0x03
};
struct __attribute__((packed)) mac_channel_assessment{
	uint8_t assessment_mode;
	uint8_t noise_level;
};
enum mac_mrf24j40_ioc{
	mac_mrf24j40_ioc_trigger_interrupt = 1,	// args = 0
	mac_mrf24j40_ioc_set_channel,			// args = (unsigned int*)
	mac_mrf24j40_ioc_channel_assessment, 	// args = (struct mac_channel_assessment*)
	mac_mrf24j40_ioc_reset
};

void 	PHY_mrf24j40_initialize(struct phy_mrf24j40* phy);

int 	MAC_mrf24j40_open(struct mac_mrf24j40* mac, struct mac_mrf24j40_open_param *init);
int 	MAC_mrf24j40_write(struct mac_mrf24j40* mac, struct mac_mrf24j40_write_param* trans, void* payload, int payloadlen);
int 	MAC_mrf24j40_read(struct mac_mrf24j40* mac, void* payload, int payload_maxlen, int timeout);
int 	MAC_mrf24j40_select(struct mac_mrf24j40* mac, int timeout);
int		MAC_mrf24j40_ioctl(struct mac_mrf24j40* mac, int request, unsigned int arguments);

int 	MAC_mrf24j40_task(struct mac_mrf24j40* mac);

#endif /* SRC_NETWORK_MAC_MAC_MRF24J40_H_ */
