/* 
 * File:   i2c_driver.h
 * Author: oussama
 *
 * Created on April 11, 2023, 1:32 PM
 */

#ifndef I2C_DRIVER_H
#define	I2C_DRIVER_H


//#if defined(CONFIG_I2C_ENABLED)

void i2c_driver_init(void);
void i2c_driver_write_byte(uint8_t devaddr, uint8_t reg, uint8_t data);
void i2c_driver_write_twobytes(uint8_t devaddr, uint8_t reg, uint16_t data);
void i2c_driver_read_byte(uint8_t devaddr, uint8_t reg, uint8_t * data);
void i2c_driver_read_twobytes(uint8_t devaddr, uint8_t reg, uint16_t * data);

//#endif /* (CONFIG_I2C_ENABLED) */

#endif	/* I2C_DRIVER_H */

