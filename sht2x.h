//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT2x Sample Code (V1.2)
// File      :  SHT2x.c
// Autor     :  MST
//              converted for PIC16F84 and PICC compiler
//              by Bartek Fabiszewski (www.fabiszewski.net)
// Brief     :  Sensor layer. Functions for sensor access
//==============================================================================

#ifndef SHT2x_H
#define SHT2x_H

//---------- Includes ----------------------------------------------------------
#define NDEBUG
#include <assert.h>

//---------- Defines -----------------------------------------------------------
//  CRC
#define        POLYNOMIAL                  0x131  //P(x)=x^8+x^5+x^4+1 = 100110001

typedef union {
    unsigned short u16;             // element specifier for accessing whole u16
    signed short i16;               // element specifier for accessing whole i16
    struct {
        // Byte-order is little endian
        byte u8L;              // element specifier for accessing low u8
        byte u8H;              // element specifier for accessing high u8
        
    } s16;                  // element spec. for acc. struct with low or high u8
} nt16;

// sensor command
#define     TRIG_T_MEASUREMENT_HM     0xE3 // command trig. temp meas. hold master
#define     TRIG_RH_MEASUREMENT_HM    0xE5 // command trig. humidity meas. hold master
#define     TRIG_T_MEASUREMENT_POLL   0xF3 // command trig. temp meas. no hold master
#define     TRIG_RH_MEASUREMENT_POLL  0xF5 // command trig. humidity meas. no hold master
#define     USER_REG_W                0xE6 // command writing user register
#define     USER_REG_R                0xE7 // command reading user register
#define     SOFT_RESET                0xFE // command soft reset

#define     SHT2x_RES_12_14BIT        0x00 // RH=12bit, T=14bit
#define     SHT2x_RES_8_12BIT         0x01 // RH= 8bit, T=12bit
#define     SHT2x_RES_10_13BIT        0x80 // RH=10bit, T=13bit
#define     SHT2x_RES_11_11BIT        0x81 // RH=11bit, T=11bit
#define     SHT2x_RES_MASK            0x81 // Mask for res. bits (7,0) in user reg.

#define     SHT2x_EOB_ON              0x40 // end of battery
#define     SHT2x_EOB_MASK            0x40 // Mask for EOB bit(6) in user reg.

#define     SHT2x_HEATER_ON           0x04 // heater on
#define     SHT2x_HEATER_OFF          0x00 // heater off
#define     SHT2x_HEATER_MASK         0x04 // Mask for Heater bit(2) in user reg.

// measurement signal selection
#define     HUMIDITY                  0x01
#define     TEMP                      0x02

#define     I2C_ADR_W                 128 // sensor I2C address + write bit
#define     I2C_ADR_R                 129 // sensor I2C address + read bit

//testing
#define     E24XX_W                   160 // 24xx00 eeprom address + write bit (testing)
#define     E24XX_R                   161 // 24xx00 eeprom address + read bit (testing)

// I2C acknowledge
#define     ACK                       0
#define     NO_ACK                    1

#define ACK_ERROR                     0x01
#define TIME_OUT_ERROR                0x02
#define CHECKSUM_ERROR                0x04
#define UNIT_ERROR                    0x08

#define SENSOR_TYPE                   0x02 // sht2x

//==============================================================================
byte sht2x_check_crc(byte data[], byte nbrOfBytes, byte checksum);
//==============================================================================
// calculates checksum for n bytes of data and compares it with expected
// checksum
// input:  data[]       checksum is built based on this data
//         nbrOfBytes   checksum is built for n bytes of data
//         checksum     expected checksum
// return: error:       CHECKSUM_ERROR = checksum does not match
//                      0              = checksum matches

//==============================================================================
byte sht2x_read_userregister(byte *pRegisterValue);
//==============================================================================
// reads the SHT2x user register (8bit)
// input : -
// output: *pRegisterValue
// return: error

//==============================================================================
byte sht2x_write_userregister(byte *pRegisterValue);
//==============================================================================
// writes the SHT2x user register (8bit)
// input : *pRegisterValue
// output: -
// return: error

//==============================================================================
void sht2x_read_buf(byte mode,byte start);
//==============================================================================
// measures humidity and temperature into scratchpad
// added by Bartek Fabiszewski

//==============================================================================
byte sht2x_measure_poll(byte eSHT2xMeasureType, nt16 *pMeasurand);
//==============================================================================
// measures humidity or temperature. This function polls every 10ms until
// measurement is ready.
// input:  eSHT2xMeasureType
// output: *pMeasurand:  humidity / temperature as raw value
// return: error
// note:   timing for timeout may be changed

//==============================================================================
byte sht2x_measure_hm(byte eSHT2xMeasureType, nt16 *pMeasurand);
//==============================================================================
// measures humidity or temperature. This function waits for a hold master until
// measurement is ready or a timeout occurred.
// input:  eSHT2xMeasureType
// output: *pMeasurand:  humidity / temperature as raw value
// return: error
// note:   timing for timeout may be changed

//==============================================================================
byte sht2x_soft_reset();
//==============================================================================
// performs a reset
// input:  -
// output: -
// return: error

//==============================================================================
float sht2x_calcRH(unsigned int u16sRH);
//==============================================================================
// calculates the relative humidity
// input:  sRH: humidity raw value (16bit scaled)
// return: pHumidity relative humidity [%RH]

//==============================================================================
float sht2x_calc_temp(unsigned int u16sT);
//==============================================================================
// calculates temperature
// input:  sT: temperature raw value (16bit scaled)
// return: temperature [∞C]

//==============================================================================
byte sht2x_get_SN(byte u8SerialNumber[]);
//==============================================================================
// gets serial number of SHT2x according application note "How To
// Read-Out the Serial Number"
// note:   readout of this function is not CRC checked
//
// input:  -
// output: u8SerialNumber: Array of 8 bytes (64Bits)
//         MSB                                         LSB
//         u8SerialNumber[7]             u8SerialNumber[0]
//         SNA_1 SNA_0 SNB_3 SNB_2 SNB_1 SNB_0 SNC_1 SNC_0
// return: error

// Common I2C Routines
//==============================================================================
void i2c_init(void);
//==============================================================================
//Initializes the ports for I2C interface

//==============================================================================
void i2c_start_condition(void);
//==============================================================================
// writes a start condition on I2C-bus
// input : -
// output: -
// return: -
// note  : timing (delay) may have to be changed for different microcontroller
//       _____
// SDA:       |_____
//       _______
// SCL :        |___

//==============================================================================
void i2c_stop_condition(void);
//==============================================================================
// writes a stop condition on I2C-bus
// input : -
// output: -
// return: -
// note  : timing (delay) may have to be changed for different microcontroller
//              _____
// SDA:   _____|
//            _______
// SCL :  ___|

//===========================================================================
byte i2c_write_byte(byte txByte);
//===========================================================================
// writes a byte to I2C-bus and checks acknowledge
// input:  txByte  transmit byte
// output: -
// return: error
// note: timing (delay) may have to be changed for different microcontroller

//===========================================================================
byte i2c_read_byte(byte ack);
//===========================================================================
// reads a byte on I2C-bus
// input:  rxByte  receive byte
// output: rxByte
// note: timing (delay) may have to be changed for different microcontroller
void i2c_sck_low(void);
void i2c_data_low(void);
void read_24aa00(void);

#endif

