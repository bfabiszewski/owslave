/*
 * sht1x.h
 *
 *  created: April 2011
 *  author: Bartek Fabiszewski
 */

#ifndef SHT1X_H_
#define SHT1X_H_

#include <math.h> // for dew point calculations

typedef union {
    unsigned int i;
    float f;
} value;

//----------------------------------------------------------------------------------
// modul-var
//----------------------------------------------------------------------------------
enum {
    TEMP, HUMI
};

#define noACK 0
#define ACK   1
//adr  command  r/w
#define STATUS_REG_W 0x06   //000   0011    0
#define STATUS_REG_R 0x07   //000   0011    1
#define MEASURE_TEMP 0x03   //000   0001    1
#define MEASURE_HUMI 0x05   //000   0010    1
#define RESET        0x1e   //000   1111    0

#define SENSOR_TYPE     0x1    // sht1x, sht7x

char sht1x_write_byte(byte value);
char sht1x_read_byte(byte ack);
void sht1x_transstart(void);
void sht1x_connectionreset(void);
char sht1x_softreset(void);
char sht1x_read_statusreg(byte *p_value, byte *p_checksum);
char sht1x_write_statusreg(byte *p_value);
char sht1x_measure(byte *p_value, byte *p_checksum, byte mode);
char sht1x_measure_buf(byte *p_buffer, byte mode);
void sht1x_read_buf(void);
void calc_sth11(float *p_humidity, float *p_temperature);
float calc_dewpoint(float h, float t);

#endif /* SHT1X_H_ */

