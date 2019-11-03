/*
 *    config.h
 *
 *    created: April 2011
 *    author: Bartek Fabiszewski (www.fabiszewski.net)
 *    version 0.3 sht2x through i2c added
 *    version 0.2 sleep added
 *    version 0.1 first release with support for sht1x
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include <htc.h>

#define _XTAL_FREQ    10000000 // 10MHz
#define byte unsigned char

// leave uncommented only necessary module
// only one module may be compiled into hex due to PIC code space limits
//#define SHT1x  // leave uncommented if you have sht1x or sht7x compatible sensor connected to pic
#define SHT2x     // leave uncommented if you have sht2x sensor supporting i2c connected to pic

#define TRUE                1
#define FALSE               0
#define INPUT               1
#define OUTPUT              0
#define HIGH                1
#define LOW                 0

#define OW                  RB0          //pin 6 for 1-wire data (this can't be changed)
#define TRIS_OW             TRISB0

#define MOD1                             //my first design used different pins

#ifdef MOD1
#define LEDMOD // leds for debugging, comment out here if not needed
#ifdef    LEDMOD
#define   LED1              RB3          //pin 9
#define   LED2              RB4          //pin 10
#define   TRIS_LED1         TRISB3
#define   TRIS_LED2         TRISB4
#endif //LEDMOD
#define DATA                RB6          //pin 12 for sht data
#define SCK                 RB7          //pin 13 for sht clock
#define TRIS_DATA           TRISB6
#define TRIS_SCK            TRISB7
#else
#define DATA                RB4          //pin 10 for sht data
#define SCK                 RB5          //pin 11 for sht clock
#define TRIS_DATA           TRISB4
#define TRIS_SCK            TRISB5
#endif //MOD1

// some global vars
byte ow_error;      // error on ow line
byte sensor_error;  // error on sensor line
byte scratchpad[6]; // 6-byte buffer [h-msb h-lsb h-crc t-msb t-lsb t-crc]


#include "1wire.h"
#ifdef SHT1x
#include "sht1x.h"
#endif
#ifdef SHT2x
#include "sht2x.h"
#endif

#endif /* CONFIG_H_ */
