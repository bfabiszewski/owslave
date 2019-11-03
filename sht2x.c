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

//---------- Includes ----------------------------------------------------------
#include "config.h"
#ifdef SHT2x
//==============================================================================
byte sht2x_check_crc(byte data[], byte nbrOfBytes, byte checksum)
//==============================================================================
{
    byte crc = 0;
    byte byteCtr;
    byte i;
    //calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
    {
        crc ^= (data[byteCtr]);
        for (i = 8; i > 0; --i)
        {
            if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
            else crc = (crc << 1);
        }
    }
    if (crc != checksum) return CHECKSUM_ERROR;
    else return 0;
}

//===========================================================================
byte sht2x_read_userregister(byte *pRegisterValue)
//===========================================================================
{
    byte checksum;   //variable for checksum byte
    byte error = 0;    //variable for error code
    
    i2c_start_condition();
    error |= i2c_write_byte (I2C_ADR_W);
    error |= i2c_write_byte (USER_REG_R);
    i2c_start_condition();
    error |= i2c_write_byte (I2C_ADR_R);
    *pRegisterValue = i2c_read_byte(ACK);
    checksum = i2c_read_byte(NO_ACK);
    error |= sht2x_check_crc (pRegisterValue,1,checksum);
    i2c_stop_condition();
    return error;
}

//===========================================================================
byte sht2x_write_userregister(byte *pRegisterValue)
//===========================================================================
{
    byte error = 0;   //variable for error code
    
    i2c_start_condition();
    error |= i2c_write_byte (I2C_ADR_W);
    error |= i2c_write_byte (USER_REG_W);
    error |= i2c_write_byte (*pRegisterValue);
    i2c_stop_condition();
    return error;
}
//----------------------------------------------------------------------------------
void sht2x_read_buf(byte mode, byte start)
//----------------------------------------------------------------------------------
// makes a measurement (humidity/temperature) with checksum
// and loads data to buffer for sending
// added by Bartek Fabiszewski
{
    //byte  checksum;   //checksum
    //byte  data[2];    //data array for checksum verification
    unsigned int i;     //counting variable
    
    //-- write I2C sensor address and command --
    i2c_start_condition();
    sensor_error |= i2c_write_byte (I2C_ADR_W); // I2C Adr
    sensor_error |= i2c_write_byte (mode);
    //-- wait until hold master is released --
    i2c_start_condition();
    sensor_error |= i2c_write_byte (I2C_ADR_R);
    TRIS_SCK = INPUT;                  // set SCL I/O port as input
    for (i = 0; i < 1000; i++)         // wait until master hold is released or
    {
        __delay_ms(1);    // a timeout (~1s) is reached
        if (SCK) break;
    }
    //-- check for timeout --
    if (SCK == LOW) sensor_error |= TIME_OUT_ERROR;
    
    //-- read two data bytes and one checksum byte --
    scratchpad[0+start] = i2c_read_byte(ACK);
    scratchpad[1+start] = i2c_read_byte(ACK);
    scratchpad[2+start] = i2c_read_byte(NO_ACK);
    
    //-- verify checksum --
    //sensor_error |= sht2x_check_crc (data,2,checksum); // FIXME: currently there is no space left on pic to implement this check
    // checking crc must be done on 1-wire master
    i2c_stop_condition();
    
}
// this function was just for testing i2c communication with simple eeprom read and write
// added by Bartek Fabiszewski
void read_24aa00(void)
{
    i2c_start_condition();
    sensor_error |= i2c_write_byte (E24XX_W);
    sensor_error |= i2c_write_byte (0);
    sensor_error |= i2c_write_byte (85);
    i2c_stop_condition();
    __delay_ms(1000);
    
    i2c_start_condition();
    sensor_error |= i2c_write_byte (E24XX_W);
    sensor_error |= i2c_write_byte (0);
    i2c_start_condition();
    sensor_error |= i2c_write_byte (E24XX_R);
    scratchpad[0] = i2c_read_byte(NO_ACK);
    i2c_stop_condition();
    
}
//===========================================================================
byte sht2x_measure_hm(byte eSHT2xMeasureType, nt16 *pMeasurand)
//===========================================================================
{
    byte  checksum;   //checksum
    byte  data[2];    //data array for checksum verification
    byte  error = 0;  //error variable
    unsigned int   i; //counting variable
    
    //-- write I2C sensor address and command --
    i2c_start_condition();
    error |= i2c_write_byte (I2C_ADR_W); // I2C Adr
    switch(eSHT2xMeasureType)
    {
        case HUMIDITY:
            error |= i2c_write_byte (TRIG_RH_MEASUREMENT_HM);
            break;
        case TEMP:
            error |= i2c_write_byte (TRIG_T_MEASUREMENT_HM);
            break;
        default:
            assert(0);
    }
    //-- wait until hold master is released --
    i2c_start_condition();
    error |= i2c_write_byte (I2C_ADR_R);
    TRIS_SCK = INPUT;                 // set SCL I/O port as input
    for(i = 0; i < 1000; i++)         // wait until master hold is released or
    { __delay_us(1000);    // a timeout (~1s) is reached
        if (SCK == 1) break;
    }
    //-- check for timeout --
    if (SCK == 0) error |= TIME_OUT_ERROR;
    
    //-- read two data bytes and one checksum byte --
    pMeasurand->s16.u8H = data[0] = i2c_read_byte(ACK);
    pMeasurand->s16.u8L = data[1] = i2c_read_byte(ACK);
    checksum = i2c_read_byte(NO_ACK);
    
    //-- verify checksum --
    error |= sht2x_check_crc (data,2,checksum);
    i2c_stop_condition();
    return error;
}

//===========================================================================
byte sht2x_measure_poll(byte eSHT2xMeasureType, nt16 *pMeasurand)
//===========================================================================
{
    byte  checksum;   //checksum
    byte  data[2];    //data array for checksum verification
    byte  error = 0;    //error variable
    unsigned int i = 0;        //counting variable
    
    //-- write I2C sensor address and command --
    i2c_start_condition();
    error |= i2c_write_byte (I2C_ADR_W); // I2C Adr
    switch(eSHT2xMeasureType)
    {
        case HUMIDITY: error |= i2c_write_byte (TRIG_RH_MEASUREMENT_POLL); break;
        case TEMP    : error |= i2c_write_byte (TRIG_T_MEASUREMENT_POLL);  break;
        default: assert(0);
    }
    //-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
    do
    {
        i2c_start_condition();
        __delay_ms(10);  //delay 10ms
        if (i++ >= 20) break;
    } while (i2c_write_byte (I2C_ADR_R) == ACK_ERROR);
    if (i >= 20) error |= TIME_OUT_ERROR;
    
    //-- read two data bytes and one checksum byte --
    pMeasurand->s16.u8H = data[0] = i2c_read_byte(ACK);
    pMeasurand->s16.u8L = data[1] = i2c_read_byte(ACK);
    checksum = i2c_read_byte(NO_ACK);
    
    //-- verify checksum --
    error |= sht2x_check_crc (data,2,checksum);
    i2c_stop_condition();
    
    return error;
}

//===========================================================================
byte sht2x_soft_reset()
//===========================================================================
{
    byte  error = 0;           //error variable
    
    i2c_start_condition();
    error |= i2c_write_byte (I2C_ADR_W); // I2C Adr
    error |= i2c_write_byte (SOFT_RESET); // Command
    i2c_stop_condition();
    
    __delay_ms(15); // wait till sensor has restarted
    
    return error;
}

//==============================================================================
float sht2x_calcRH(unsigned int u16sRH)
//==============================================================================
{
    float humidityRH;              // variable for result
    
    u16sRH &= ~0x0003;          // clear bits [1..0] (status bits)
    //-- calculate relative humidity [%RH] --
    
    humidityRH = -6.0 + 125.0/65536 * (float)u16sRH; // RH= -6 + 125 * SRH/2^16
    return humidityRH;
}

//==============================================================================
float sht2x_calc_temp(unsigned int u16sT)
//==============================================================================
{
    float temperatureC;            // variable for result
    
    u16sT &= ~0x0003;           // clear bits [1..0] (status bits)
    
    //-- calculate temperature [∞C] --
    temperatureC= -46.85 + 175.72/65536 *(float)u16sT; //T= -46.85 + 175.72 * ST/2^16
    return temperatureC;
}

//==============================================================================
byte sht2x_get_SN(byte u8SerialNumber[])
//==============================================================================
{
    byte  error = 0;                          //error variable
    
    //Read from memory location 1
    i2c_start_condition();
    error |= i2c_write_byte (I2C_ADR_W);    //I2C address
    error |= i2c_write_byte (0xFA);         //Command for readout on-chip memory
    error |= i2c_write_byte (0x0F);         //on-chip memory address
    i2c_start_condition();
    error |= i2c_write_byte (I2C_ADR_R);    //I2C address
    u8SerialNumber[5] = i2c_read_byte(ACK); //Read SNB_3
    i2c_read_byte(ACK);                     //Read CRC SNB_3 (CRC is not analyzed)
    u8SerialNumber[4] = i2c_read_byte(ACK); //Read SNB_2
    i2c_read_byte(ACK);                     //Read CRC SNB_2 (CRC is not analyzed)
    u8SerialNumber[3] = i2c_read_byte(ACK); //Read SNB_1
    i2c_read_byte(ACK);                     //Read CRC SNB_1 (CRC is not analyzed)
    u8SerialNumber[2] = i2c_read_byte(ACK); //Read SNB_0
    i2c_read_byte(NO_ACK);                  //Read CRC SNB_0 (CRC is not analyzed)
    i2c_stop_condition();
    
    //Read from memory location 2
    i2c_start_condition();
    error |= i2c_write_byte (I2C_ADR_W);    //I2C address
    error |= i2c_write_byte (0xFC);         //Command for readout on-chip memory
    error |= i2c_write_byte (0xC9);         //on-chip memory address
    i2c_start_condition();
    error |= i2c_write_byte (I2C_ADR_R);    //I2C address
    u8SerialNumber[1] = i2c_read_byte(ACK); //Read SNC_1
    u8SerialNumber[0] = i2c_read_byte(ACK); //Read SNC_0
    i2c_read_byte(ACK);                     //Read CRC SNC0/1 (CRC is not analyzed)
    u8SerialNumber[7] = i2c_read_byte(ACK); //Read SNA_1
    u8SerialNumber[6] = i2c_read_byte(ACK); //Read SNA_0
    i2c_read_byte(NO_ACK);                  //Read CRC SNA0/1 (CRC is not analyzed)
    i2c_stop_condition();
    
    return error;
}

// common i2c routines

//==============================================================================
void i2c_init(void)
//==============================================================================
{
    TRIS_DATA = OUTPUT;               // Set port as output for configuration
    TRIS_SCK = OUTPUT;                // Set port as output for configuration
    
    DATA = LOW;                       // Set SDA level as low for output mode
    SCK = LOW;                        // Set SCL level as low for output mode
    
    TRIS_DATA = INPUT;                // I2C-bus idle mode SDA released (input)
    TRIS_SCK = INPUT;                 // I2C-bus idle mode SCL released (input)
}

//==============================================================================
void i2c_start_condition(void)
//==============================================================================
{
    TRIS_DATA = INPUT;
    TRIS_SCK = INPUT;
    i2c_data_low();
    __delay_us(10);  // hold time start condition (t_HD;STA)
    i2c_sck_low();
    __delay_us(10);
    TRIS_DATA = INPUT;
    TRIS_SCK = INPUT;
}

//==============================================================================
void i2c_stop_condition(void)
//==============================================================================
{
    i2c_data_low();
    i2c_sck_low();
    TRIS_SCK = INPUT;
    __delay_us(10);  // set-up time stop condition (t_SU;STO)
    TRIS_DATA = INPUT;
    __delay_us(10);
}

//==============================================================================
byte i2c_write_byte(byte txByte)
//==============================================================================
{
    byte mask, error = 0;
    for (mask = 0x80; mask > 0; mask >>= 1)   //shift bit for masking (8 times)
    {
        if ((mask & txByte) == 0) {
            i2c_data_low();        //masking txByte, write bit to SDA-Line
        }
        else
        {
            TRIS_DATA = INPUT;
        }
        __delay_us(1);             //data set-up time (t_SU;DAT)
        TRIS_SCK = INPUT;          //generate clock pulse on SCL
        __delay_us(5);             //SCL high time (t_HIGH)
        i2c_sck_low();
        __delay_us(1);             //data hold time(t_HD;DAT)
    }
    TRIS_DATA = INPUT;             //release SDA-line
    TRIS_SCK = INPUT;              //clk #9 for ack
    __delay_us(1);                 //data set-up time (t_SU;DAT)
    if (DATA) error = ACK_ERROR;      //check ack from i2c slave
    i2c_sck_low();
    __delay_us(20);                //wait time to see byte package on scope
    return error;                  //return error code
}

//==============================================================================
byte i2c_read_byte(byte ack)
//==============================================================================
{
    byte mask, rxByte = 0;
    TRIS_DATA = INPUT;           //release SDA-line
    for (mask = 0x80; mask > 0; mask >>= 1)   //shift bit for masking (8 times)
    {
        TRIS_SCK = INPUT;        //start clock on SCL-line
        //__delay_us(1);         //data set-up time (t_SU;DAT)
        //__delay_us(3);         //SCL high time (t_HIGH)
        __delay_us(4);
        if (DATA) rxByte=(rxByte | mask); //read bit
        i2c_sck_low();
        __delay_us(1);           //data hold time(t_HD;DAT)
    }
    if (!ack) { i2c_data_low(); }//send acknowledge if necessary
    __delay_us(1);               //data set-up time (t_SU;DAT)
    TRIS_SCK = INPUT;            //clk #9 for ack
    __delay_us(5);               //SCL high time (t_HIGH)
    i2c_sck_low();
    TRIS_DATA = INPUT;           //release SDA-line
    __delay_us(20);              //wait time to see byte package on scope
    return rxByte;               //return error code
}
#endif

// functions to save some space on pic
// added by Bartek Fabiszewski
void i2c_sck_low(void)
{
    TRIS_SCK = OUTPUT;
    SCK = LOW;
}
void i2c_data_low(void)
{
    TRIS_DATA = OUTPUT;
    DATA = LOW;
}

