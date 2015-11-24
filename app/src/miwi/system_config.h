/********************************************************************
 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PIC(R) Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *******************************************************************/

#ifndef _SYSTEM_CONFIG_H
    #define _SYSTEM_CONFIG_H
 
#include "miwi_config.h"        //Include miwi application layer configuration file
#include "miwi_config_mesh.h"   //Include protocol layer configuration file
#include "config_24j40.h"       //Transceiver configuration file
#include "rf_port.h" 
#include <debug.h>
#include <semaphore.h>
// There are three ways to use NVM to store data: External EPROM, Data EEPROM and
// programming space, with following definitions:
//      #define USE_EXTERNAL_EEPROM
//      #define USE_DATA_EEPROM
//      #define USE_PROGRAMMING_SPACE
// Each demo board has defined the method of using NVM, as
// required by Network Freezer feature.
//#define USE_EXTERNAL_EEPROM

//#define SUPPORT_TWO_SPI

// Define EEPROM_SHARE_SPI if external EEPROM shares the SPI
// bus with RF transceiver
//#define EEPROM_SHARE_SPI


// MRF24J40 Pin Definitions


// EEProm Pin Definitions

// SPI1 Pin Definitions

// SPI2 Pin Definitions

// Switch and LED Pin Definitions

// External EEPROM Pin Definitions
//#define EE_nCS_TRIS         TRISDbits.TRISD5

//External SST Serial Flash Definitions

// LCD Pin Definitions

#define TMRL    (rf_port_get_sys_count() & 0xFF)
#define ROM
#define delay_ms(x)    usleep_s(1000*x)

#define CONSOLE_PutString   LREP
#define Printf              LREP
#define CONSOLE_PrintDec(x) LREP("%d", x)
#define CONSOLE_PrintHex(x) LREP("%X", x)
#define CONSOLE_Put(x)      LREP("%c", x)

#define lock_define(obj)	sem_t g_sem_##obj = 0;
#define lock_init(obj)		if(!g_sem_##obj) sem_init(&g_sem_##obj, 0, 1);
#define lock(obj, x)		sem_wait(&g_sem_##obj); x; sem_post(&g_sem_##obj);
	
#endif
