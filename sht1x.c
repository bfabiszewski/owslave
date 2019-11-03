/***********************************************************************************
 Project:          SHT1x/7x demo program (V2.4)
 Filename:         SHT1x_sample_code.c
 
 Autor:            MST
                   converted for PIC16F84 and PICC compiler
                   by Bartek Fabiszewski (www.fabiszewski.net)
 
 Copyright:        (c) Sensirion AG
 ***********************************************************************************/
// Revisions:
// V2.4     calc_sht11()    Coefficients for humidity and temperature conversion
//                          changed (for V4 sensors)
//       calc_dewpoint()    New formula for dew point calculation


#include "config.h"
#ifdef SHT1x

//----------------------------------------------------------------------------------
char sht1x_write_byte(byte value)
//----------------------------------------------------------------------------------
// writes a byte on the Sensibus and checks the acknowledge
{
    byte i, error = 0;
    TRIS_DATA = OUTPUT;
    for (i = 0x80; i > 0; i /= 2) //shift bit for masking
    {
        if (i & value)
            DATA = 1; //masking value with i , write to SENSI-BUS
        else
            DATA = 0;
        __delay_us(1); //observe setup time
        SCK = 1; //clk for SENSI-BUS
        __delay_us(5); //pulswith approx. 5 us
        SCK = 0;
        __delay_us(1); //observe hold time
    }
    TRIS_DATA = INPUT; //release DATA-line
    __delay_us(1); //observe setup time
    SCK = 1; //clk #9 for ack
    error = DATA; //check ack (DATA will be pulled down by SHT11)
    SCK = 0;
    return error; //error=1 in case of no acknowledge
}

//----------------------------------------------------------------------------------
char sht1x_read_byte(byte ack)
//----------------------------------------------------------------------------------
// reads a byte form the Sensibus and gives an acknowledge in case of "ack=1"
{
    byte i, val = 0;
    TRIS_DATA = INPUT; //release DATA-line
    for (i = 0x80; i > 0; i /= 2) //shift bit for masking
    {
        SCK = 1; //clk for SENSI-BUS
        if (DATA)
            val = (val | i); //read bit
        SCK = 0;
    }
    TRIS_DATA = OUTPUT;
    DATA = !ack; //in case of "ack == 1" pull down DATA-Line
    __delay_us(1); //observe setup time
    SCK = 1; //clk #9 for ack
    __delay_us(5); //pulswith approx. 5 us
    SCK = 0;
    __delay_us(1); //observe hold time
    TRIS_DATA = INPUT; //release DATA-line
    return val;
}

//----------------------------------------------------------------------------------
void sht1x_transstart(void)
//----------------------------------------------------------------------------------
// generates a transmission start
//       _____         ________
// DATA:      |_______|
//           ___     ___
// SCK : ___|   |___|   |______
{
    TRIS_DATA = OUTPUT;
    TRIS_SCK = OUTPUT;
    DATA = 1;
    SCK = 0; //Initial state
    __delay_us(1);
    SCK = 1;
    __delay_us(1);
    DATA = 0;
    __delay_us(1);
    SCK = 0;
    __delay_us(5);
    SCK = 1;
    __delay_us(1);
    DATA = 1;
    __delay_us(1);
    SCK = 0;
}

//----------------------------------------------------------------------------------
void sht1x_connectionreset(void)
//----------------------------------------------------------------------------------
// communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
//       _____________________________________________________         ________
// DATA:                                                      |_______|
//          _    _    _    _    _    _    _    _    _        ___     ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
{
    byte i;
    TRIS_DATA = OUTPUT;
    DATA = 1;
    SCK = 0; //Initial state
    for (i = 0; i < 9; i++) //9 SCK cycles
    {
        SCK = 1;
        SCK = 0;
    }
    sht1x_transstart(); //transmission start
}

//----------------------------------------------------------------------------------
char sht1x_softreset(void)
//----------------------------------------------------------------------------------
// resets the sensor by a softreset
{
    byte error = 0;
    sht1x_connectionreset(); //reset communication
    error += sht1x_write_byte(RESET); //send RESET-command to sensor
    return error; //error=1 in case of no response form the sensor
}

//----------------------------------------------------------------------------------
char sht1x_read_statusreg(byte *p_value, byte *p_checksum)
//----------------------------------------------------------------------------------
// reads the status register with checksum (8-bit)
{
    byte error = 0;
    sht1x_transstart(); //transmission start
    error = sht1x_write_byte(STATUS_REG_R); //send command to sensor
    *p_value = sht1x_read_byte(ACK); //read status register (8-bit)
    *p_checksum = sht1x_read_byte(noACK); //read checksum (8-bit)
    return error; //error=1 in case of no response form the sensor
}

//----------------------------------------------------------------------------------
char sht1x_write_statusreg(byte *p_value)
//----------------------------------------------------------------------------------
// writes the status register with checksum (8-bit)
{
    byte error = 0;
    sht1x_transstart(); //transmission start
    error += sht1x_write_byte(STATUS_REG_W);//send command to sensor
    error += sht1x_write_byte(*p_value); //send value of status register
    return error; //error >= 1 in case of no response form the sensor
}

//----------------------------------------------------------------------------------
char sht1x_measure(byte *p_value, byte *p_checksum,
                   byte mode)
//----------------------------------------------------------------------------------
// makes a measurement (humidity/temperature) with checksum
{
    byte error = 0;
    unsigned int i;
    
    sht1x_transstart(); //transmission start
    switch (mode) { //send command to sensor
        case TEMP:
            error += sht1x_write_byte(MEASURE_TEMP);
            break;
        case HUMI:
            error += sht1x_write_byte(MEASURE_HUMI);
            break;
        default:
            break;
    }
    for (i = 0; i < 65535; i++)
        if (DATA == 0)
            break; //wait until sensor has finished the measurement
    if (DATA)
        error += 1; // or timeout (~2 sec.) is reached
    *(p_value) = sht1x_read_byte(ACK); //read the first byte (MSB)
    *(p_value + 1) = sht1x_read_byte(ACK); //read the second byte (LSB)
    *p_checksum = sht1x_read_byte(noACK); //read checksum
    return error;
}


//----------------------------------------------------------------------------------
void sht1x_read_buf(void)
//----------------------------------------------------------------------------------
// makes a measurement (humidity/temperature) with checksum
// and loads data to buffer for sending
// added by Bartek Fabiszewski
{
    unsigned int i;
    sht1x_transstart(); //transmission start
    sensor_error += sht1x_write_byte(MEASURE_HUMI);
    for (i = 0; i < 65535; i++)
        if (DATA == LOW)
            break; //wait until sensor has finished the measurement
    if (DATA)
        sensor_error += 1; // or timeout (~2 sec.) is reached
    scratchpad[0] = sht1x_read_byte(ACK); //read the first byte (MSB)
    scratchpad[1] = sht1x_read_byte(ACK); //read the second byte (LSB)
    scratchpad[2] = sht1x_read_byte(noACK); //read checksum
    sht1x_transstart(); //transmission start
    sensor_error += sht1x_write_byte(MEASURE_TEMP);
    for (i = 0; i < 65535; i++)
        if (DATA == LOW)
            break; //wait until sensor has finished the measurement
    if (DATA)
        sensor_error += 1; // or timeout (~2 sec.) is reached
    scratchpad[3] = sht1x_read_byte(ACK); //read the first byte (MSB)
    scratchpad[4] = sht1x_read_byte(ACK); //read the second byte (LSB)
    scratchpad[5] = sht1x_read_byte(noACK); //read checksum
}
//----------------------------------------------------------------------------------------
void calc_sth11(float *p_humidity, float *p_temperature)
//----------------------------------------------------------------------------------------
// calculates temperature [∞C] and humidity [%RH]
// input :  humi [Ticks] (12 bit)
//          temp [Ticks] (14 bit)
// output:  humi [%RH]
//          temp [∞C]
{
    const float C1 = -2.0468; // for 12 Bit RH
    const float C2 = +0.0367; // for 12 Bit RH
    const float C3 = -0.0000015955; // for 12 Bit RH
    const float T1 = +0.01; // for 12 Bit RH
    const float T2 = +0.00008; // for 12 Bit RH
    
    float rh = *p_humidity; // rh:      Humidity [Ticks] 12 Bit
    float t = *p_temperature; // t:       Temperature [Ticks] 14 Bit
    float rh_lin; // rh_lin:  Humidity linear
    float rh_true; // rh_true: Temperature compensated humidity
    float t_C; // t_C   :  Temperature [∞C]
    
    t_C = t * 0.01 - 40.1; //calc. temperature [∞C] from 14 bit temp. ticks @ 5V
    rh_lin = C3 * rh * rh + C2 * rh + C1; //calc. humidity from ticks to [%RH]
    rh_true = (t_C - 25) * (T1 + T2 * rh) + rh_lin; //calc. temperature compensated humidity [%RH]
    if (rh_true > 100)
        rh_true = 100; //cut if the value is outside of
    if (rh_true < 0.1)
        rh_true = 0.1; //the physical possible range
    
    *p_temperature = t_C; //return temperature [∞C]
    *p_humidity = rh_true; //return humidity[%RH]
}

//--------------------------------------------------------------------
float calc_dewpoint(float h, float t)
//--------------------------------------------------------------------
// calculates dew point
// input:   humidity [%RH], temperature [∞C]
// output:  dew point [∞C]
{
    float k, dew_point;
    
    k = (log10(h) - 2) / 0.4343 + (17.62 * t) / (243.12 + t);
    dew_point = 243.12 * k / (17.62 - k);
    return dew_point;
}

/*//----------------------------------------------------------------------------------
 void main()
 //----------------------------------------------------------------------------------
 // sample program that shows how to use SHT11 functions
 // 1. connection reset
 // 2. measure humidity [ticks](12 bit) and temperature [ticks](14 bit)
 // 3. calculate humidity [%RH] and temperature [∞C]
 // 4. calculate dew point [∞C]
 // 5. print temperature, humidity, dew point
 
 {
 value humi_val, temp_val;
 float dew_point;
 byte error, checksum;
 
 sht1x_connectionreset();
 while (1) {
 error = 0;
 error += sht1x_measure((byte*) &humi_val.i, &checksum, HUMI); //measure humidity
 error += sht1x_measure((byte*) &temp_val.i, &checksum, TEMP); //measure temperature
 if (error != 0)
 sht1x_connectionreset(); //in case of an error: connection reset
 else {
 humi_val.f = (float) humi_val.i; //converts integer to float
 temp_val.f = (float) temp_val.i; //converts integer to float
 //calc_sth11(&humi_val.f,&temp_val.f);            //calculate humidity, temperature
 //dew_point=calc_dewpoint(humi_val.f,temp_val.f); //calculate dew point
 //printf("temp:%5.1fC humi:%5.1f%% dew point:%5.1fC\n",temp_val.f,humi_val.f,dew_point);
 }
 //----------wait approx. 0.8s to avoid heating up SHTxx------------------------------
 __delay_ms(800);
 ; //(be sure that the compiler doesn't eliminate this line!)
 //-----------------------------------------------------------------------------------
 }
 }*/
#endif

