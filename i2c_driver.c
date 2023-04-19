



#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "i2c_driver.h"
#include "mcc_generated_files/mcc.h"



static inline void I2C1_WaitIdle(void);
static inline void I2C1_MasterStart(void);
static inline void I2C1_MasterEnableRestart(void);
static inline void I2C1_MasterStop(void);
static inline void I2C1_MasterSendTxData(uint8_t data);
static inline uint8_t I2C1_MasterGetRxData(uint8_t ack);
static inline void I2C1_MasterStartRx(void);
static inline void I2C1_MasterStopRx(void);
static inline bool I2C1_MasterIsNack(void);
static inline void I2C1_MasterSendNack(uint8_t ack);
static inline bool I2C1_MasterIsRxBufFull(void);

/* I2C1 Register Level interfaces */
void i2c_driver_init(void)
{
    SSP1STAT = 0x00;
    SSP1CON1 = 0x08;        //i2c master mode  
    SSP1CON2 = 0x00;        //idle state
    SSP1ADD  = 0x27;        //Baud Rate Clock Divider bits
    SSP1CON1bits.SSPEN = 1;        //Enables MSSP and configures SCL & SD 
}

void i2c_driver_write_byte(uint8_t devaddr, uint8_t reg, uint8_t data)
{
  while (true)
  {
    I2C1_MasterStart();     //Start condition
    I2C1_MasterSendTxData(devaddr);     //Send address of slave device
    if (I2C1_MasterIsNack())
      continue;         //if slave add is not acknowledged jump back to start condition
    
    I2C1_MasterSendTxData(reg);     //Send address of the register to write in
    if (I2C1_MasterIsNack())
      continue;         //if register add is not acknowledged jump back to start condition

    I2C1_MasterSendTxData(data);        //Send the date
    if (I2C1_MasterIsNack())
      continue;         //if data not acknowledged jump back to start condition

    break;      //when everything is acknowledged exit
  }
  I2C1_MasterStop();        //Stop condition
}

void i2c_driver_write_twobytes(uint8_t devaddr, uint8_t reg, uint16_t data)
{
  while (true)
  {
    I2C1_MasterStart();         //Start condition
    I2C1_MasterSendTxData(devaddr);         //Send address of slave device
    if (I2C1_MasterIsNack())
      continue;         //if slave add is not acknowledged jump back to start condition
    
    I2C1_MasterSendTxData(reg);         //Send address of the register to write in
    if (I2C1_MasterIsNack())
      continue;         //if register add is not acknowledged jump back to start condition

    I2C1_MasterSendTxData((data >> 8) & 0x00FF);        //Send the 1st byte of date
    if (I2C1_MasterIsNack())
      continue;         //if data not acknowledged jump back to start condition

    I2C1_MasterSendTxData(data & 0x00FF);           //Send the 2nd byte of date
    if (I2C1_MasterIsNack())
      continue;     //if data not acknowledged jump back to start condition

    break;      //when everything is acknowledged exit
  }
  I2C1_MasterStop();         //Stop condition
}

void i2c_driver_read_byte(uint8_t devaddr, uint8_t reg, uint8_t * data)
{
  while (true)
  {
    I2C1_MasterStart();         //Start condition
    I2C1_MasterSendTxData(devaddr);              //Send address of slave device (write mode)
    if (I2C1_MasterIsNack())
      continue;         //if slave add is not acknowledged jump back to start condition
    
    I2C1_MasterSendTxData(reg);         //Send address of the register to read from
    if (I2C1_MasterIsNack())        
      continue;         //if register add is not acknowledged jump back to start condition

    I2C1_MasterEnableRestart();         //Restart condition

    I2C1_MasterSendTxData(devaddr | 0x1);       //Send address of slave device with (read mode)
    if (I2C1_MasterIsNack())
      continue;         //if slave add is not acknowledged jump back to start condition

    *data = I2C1_MasterGetRxData(true);         //Read received byte and send NACK to end 

    break;
  }
  I2C1_MasterStop();          //Stop condition
}

void i2c_driver_read_twobytes(uint8_t devaddr, uint8_t reg, uint16_t * data)
{
  while (true)
  {
      send_string("inside while\n");
    I2C1_MasterStart();
    I2C1_MasterSendTxData(devaddr);
    if (I2C1_MasterIsNack())
      continue;
        send_string("after 1st nack\n");
    I2C1_MasterSendTxData(reg);
    if (I2C1_MasterIsNack())
      continue;
    send_string("after 2st nack\n");
    I2C1_MasterEnableRestart();

    I2C1_MasterSendTxData(devaddr | 0x1);
    if (I2C1_MasterIsNack())
      continue;
    send_string("after 3st nack\n");
    *data = (I2C1_MasterGetRxData(false) << 8);
    *data += I2C1_MasterGetRxData(true);

    break;
  }
  send_string("before stop\n");
  I2C1_MasterStop();
  send_string("after stop\n");
}

static inline void I2C1_WaitIdle(void)
{
  
  while ((SSP1STATbits.RW)   || (SSP1CON2bits.SEN) || 
         (SSP1CON2bits.RSEN) || (SSP1CON2bits.PEN) || 
         (SSP1CON2bits.RCEN) || (SSP1CON2bits.ACKEN) != 0) {}       //wait until bus is idle

}

static inline void I2C1_MasterStart(void)
{
    I2C1_WaitIdle();
    SSP1CON2bits.SEN = 1;       //Initiate start condition
}

static inline void I2C1_MasterEnableRestart(void)
{
    I2C1_WaitIdle();
    SSP1CON2bits.RSEN = 1;      //Initiate Repeated Start condition
}

static inline void I2C1_MasterStop(void)
{
    I2C1_WaitIdle();
    SSP1CON2bits.PEN = 1;       //Initiate Stop condition
}

static inline void I2C1_MasterSendTxData(uint8_t data)
{
    I2C1_WaitIdle();        //wait for bus to be idle
    SSP1BUF  = data;        //push data in TX/RX buffer
}
static inline uint8_t I2C1_MasterGetRxData(uint8_t ack)
{
    uint8_t b;

    I2C1_WaitIdle();
    I2C1_MasterStartRx();
    while (!I2C1_MasterIsRxBufFull()) {}
    
    b = SSP1BUF;
    I2C1_MasterStopRx();
    I2C1_MasterSendNack(ack);
    return b;
}

/*
static inline uint8_t I2C1_MasterGetRxData(uint8_t ack)
{
    uint8_t DataByte;
    I2C1_WaitIdle();        //wait for bus to be idle
    PIR1bits.SSP1IF = 0; //clear interrupt flag
    I2C1_MasterStartRx();       //start receive
    while (!PIR1bits.SSP1IF) {}        //wait for RX buff to be full
    PIR1bits.SSP1IF = 0;
    DataByte = SSP1BUF;
    I2C1_MasterStopRx();
    I2C1_MasterSendNack(ack);
    return DataByte;
}
*/
static inline void I2C1_MasterStartRx(void)
{
    SSP1CON2bits.RCEN = 1;      //Enables Receive mode
}

static inline void I2C1_MasterStopRx(void)
{
    SSP1CON2bits.RCEN = 0;      //Receive idle
}


static inline bool I2C1_MasterIsNack(void)
{
  I2C1_WaitIdle();
  return SSP1CON2bits.ACKSTAT;      //('HIGH'= NACK received), ('LOW' = ACK received)
}

static inline void I2C1_MasterSendNack(uint8_t ack)
{
    SSP1CON2bits.ACKDT = ack;       //('HIGH'= NACK received), ('LOW' = ACK received)
    I2C1_WaitIdle();
    SSP1CON2bits.ACKEN = 1;//Initiate Acknowledge sequence on SDA and SCL, and transmit ACKDT data bit.
}

static inline bool I2C1_MasterIsRxBufFull(void)
{
    return SSP1STATbits.BF; 
}



