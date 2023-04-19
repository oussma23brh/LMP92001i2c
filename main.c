/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F46K22
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"
#include <string.h>
#include "i2c_driver.h"

/*defines*/
#define ADC_I2C_ADDRESS 0x40
#define REF_REGISTER 0x66
#define GEN_STATUS 0x10
#define GEN_CONTROL 0x14
#define ADC_CH_EN_1 0x19
#define CONV_TRIG 0x1C
#define ADC_RESULT_CH_1 0x20
#define ADC_RESULT_CH_2 0x21
#define ADC_RESULT_CH_3 0x22
#define ADC_RESULT_CH_4 0x23

/*constants*/
uint8_t adc_software_reset[2] = {GEN_CONTROL,0x80};       //resets the adc
uint8_t channel_config[2] = {ADC_CH_EN_1,0x0F};     //configure channels 0,1,2,3 as ADC inputs
uint8_t lock_adc_registers[2] = {GEN_CONTROL,0x02};        //lock the internal registers of lmp92001
uint8_t continous_conversion[2] = {GEN_CONTROL,0x01};       //enable continous conversion
uint8_t trigger_conversion[2] = {CONV_TRIG,0x00};       //trigger the conversion of the enabled channels
uint16_t ADC_result[3] = {0x0000,0x0000,0x0000,0x0000};
uint16_t ADC_result0 = 0x0FFF;
double SC_voltage = 0;       //voltage at output of signal conditioning circuit
double voltage = 0;       //Analog value
char ResultBuffer[15];      //buffer to hold characters to be displayed on terminal
char DigitalBuffer[20];
char SCBuffer[20];
 
/*prototypes*/
void i2c_driver_init(void);
void i2c_driver_write_byte(uint8_t devaddr, uint8_t reg, uint8_t data);
void i2c_driver_write_twobytes(uint8_t devaddr, uint8_t reg, uint16_t data);
void i2c_driver_read_byte(uint8_t devaddr, uint8_t reg, uint8_t * data);
void i2c_driver_read_twobytes(uint8_t devaddr, uint8_t reg, uint16_t * data);
void ADC_reset(void);
void ADC_init(void);
void ADC_read(void);

/*main function*/
void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();
    i2c_driver_init();
    send_string("Initializing the ADC...\n");
    ADC_reset();        //reset the adc
    send_string("Reset DONE! \n");
    __delay_ms(1000);       //wait for the adc to reset
    ADC_init();        //initialize the adc
    send_string("ADC ready!\n");
    //I2C1_WriteNBytes(ADC_I2C_ADDRESS, continous_conversion, 2);       //trigger continous conv
    while (1)
    {
        ADC_read();
        uint8_t channel=0;
        send_string("Result Ready!\n");
        send_string("////////////       CHANNEL1");send_string("       ////////////\n");
            sprintf(DigitalBuffer,"%x",ADC_result0);
            send_string("Digital data: ");  send_string(DigitalBuffer);  send_string("\n");
            SC_voltage = (double)(ADC_result[channel]*5)/4096;      //calculate the 
            sprintf(SCBuffer,"%.4g",SC_voltage);
            send_string("SC data: ");  send_string(SCBuffer);  send_string("\n");
            //voltage = (double)(SC_voltage - 1.024) / 0.2048;
            //sprintf(ResultBuffer, " %.3g", voltage);     //get ASCII code of voltage
            send_string("Voltage: ");  send_string(ResultBuffer);  send_string("\n");  send_string("\n");  send_string("\n");
        /*
        for(channel = 0;channel < 4;channel++){
            send_string("////////////       CHANNEL");printf("%d",channel);send_string("       ////////////\n");
            sprintf(DigitalBuffer,"%x",ADC_result[channel]);
            SC_voltage = (double)(ADC_result[channel]*2.048)/2047;      //calculate the 
            sprintf(SCBuffer,"%.4g",SC_voltage);
            voltage = (double)(SC_voltage - 1.024) / 0.2048;
            sprintf(ResultBuffer, " %.3g", voltage);     //get ASCII code of voltage
            send_string("Digital data: ");  send_string(DigitalBuffer);  send_string("\n");
            send_string("SC data: ");  send_string(SCBuffer);  send_string("\n");
            send_string("Voltage: ");  send_string(ResultBuffer);  send_string("\n");  send_string("\n");  send_string("\n");
        }
        */
        __delay_ms(1000);
    }
}

void ADC_init(void){
    i2c_driver_write_byte(ADC_I2C_ADDRESS, ADC_CH_EN_1, 0x0F);      //configure ADC channels
    i2c_driver_write_byte(ADC_I2C_ADDRESS, GEN_CONTROL, 0x02);       //lock the internal registers
}


void ADC_reset(void){
    send_string("Start of reset \n");
    i2c_driver_write_byte(ADC_I2C_ADDRESS,GEN_CONTROL,0x80);  //send reset command words
    //I2C1_Write1ByteRegister(ADC_I2C_ADDRESS, GEN_CONTROL, 0x80);        //send reset command words
    send_string("End of reset \n");
}

void ADC_read(void){
    i2c_driver_write_byte(ADC_I2C_ADDRESS, GEN_CONTROL, 0x02);       //lock the internal registers
    i2c_driver_write_byte(ADC_I2C_ADDRESS,CONV_TRIG, 0x00);        //trigger conversion
    uint8_t ADC_data[2];        //buffer to temporarily hold conversion results
    uint8_t busy_status;
    do{
        i2c_driver_read_byte(ADC_I2C_ADDRESS,GEN_STATUS,&busy_status);
    }while(busy_status & (1<<7) !=0);
    i2c_driver_write_byte(ADC_I2C_ADDRESS, GEN_CONTROL, 0x00);       //unlock the internal registers
    //reading the conversion result of ch1
    i2c_driver_read_twobytes(ADC_I2C_ADDRESS, ADC_RESULT_CH_1, &ADC_result0);
    //ADC_result[0] = i2c_driver_read_twobytes(ADC_I2C_ADDRESS, ADC_RESULT_CH_1, uint16_t * data);       //  
    
    /*
    //reading the conversion result of ch2
    I2C1_WriteNBytes(ADC_I2C_ADDRESS,ADC_RESULT_CH_2,1);      //prepare to read conversion results of ch2
    I2C1_ReadNBytes(ADC_I2C_ADDRESS,ADC_data,2);       //read 2 bytes (ch 2) store them in ADC_data
    ADC_result[1] = (ADC_data[0] << 8) | ADC_data[1];
    
    //reading the conversion result of ch3
    I2C1_WriteNBytes(ADC_I2C_ADDRESS,ADC_RESULT_CH_3,1);      //prepare to read conversion results of ch3
    I2C1_ReadNBytes(ADC_I2C_ADDRESS,ADC_data,2);       //read 2 bytes (ch 3) store them in ADC_data
    ADC_result[2] = (ADC_data[0] << 8) | ADC_data[1];
    
    //reading the conversion result of ch4
    I2C1_WriteNBytes(ADC_I2C_ADDRESS,ADC_RESULT_CH_4,1);      //prepare to read conversion results of ch4
    I2C1_ReadNBytes(ADC_I2C_ADDRESS,ADC_data,2);       //read 2 bytes (ch 4) store them in ADC_data
    ADC_result[3] = (ADC_data[0] << 8) | ADC_data[1];
    */
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //untested alternative
    /*
    uint8_t channel = 0;
    uint8_t ch_address = ADC_RESULT_CH_1;       //address of channel to read from
    for(channel = 0; channel < 4; channel++){
        I2C2_WriteNBytes(ADC_I2C_ADDRESS,ch_address,1);      //prepare to read conversion results of chx
        I2C2_ReadNBytes(ADC_I2C_ADDRESS,ADC_data,2);       //read 2 bytes (chx) store them in ADC_data
        ADC_result[channel] = (ADC_data[0] << 8) | ADC_data[1];
        ch_address++;
    }
    */
}


/**
 End of File
*/