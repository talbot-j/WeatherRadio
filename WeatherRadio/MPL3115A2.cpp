
/*-----------------------------------------------*/
/** @addtogroup mpl3115a2_device MPL3115A2 Barometric/Pressure Sensor
 * @{
 *
 * @file MPL3115A2.cpp
 *
 * @author     Joshua R. Talbot
 *
 * @date       31JAN17
 * 
 * @license
 * MIT License
 * 
 * Copyright (c) 2018 J. Talbot
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "Arduino.h"
#include <Wire.h>

#include "MPL3115A2.h"

/*-----------------------------------------*/
/* Private declarations  */

#define MPL3115A2_ADDRESS_7BIT                  (0x60)    // 1100000
#define MPL3115A2_ADDRESS_8BIT_READ             (0xC1)
#define MPL3115A2_ADDRESS_8BIT_WRITE            (0xC0)

#define MPL3115A2_ADDRESS                       MPL3115A2_ADDRESS_7BIT

#define MPL3115A2_REGISTER_STATUS               (0x00)
#define MPL3115A2_REGISTER_STATUS_TDR           (0x02)
#define MPL3115A2_REGISTER_STATUS_PDR           (0x04)
#define MPL3115A2_REGISTER_STATUS_PTDR          (0x08)

#define MPL3115A2_REGISTER_PRESSURE_MSB         (0x01)
#define MPL3115A2_REGISTER_PRESSURE_CSB         (0x02)
#define MPL3115A2_REGISTER_PRESSURE_LSB         (0x03)

#define MPL3115A2_REGISTER_TEMP_MSB             (0x04)
#define MPL3115A2_REGISTER_TEMP_LSB             (0x05)

#define MPL3115A2_REGISTER_DR_STATUS            (0x06)

#define MPL3115A2_OUT_P_DELTA_MSB               (0x07)
#define MPL3115A2_OUT_P_DELTA_CSB               (0x08)
#define MPL3115A2_OUT_P_DELTA_LSB               (0x09)

#define MPL3115A2_OUT_T_DELTA_MSB               (0x0A)
#define MPL3115A2_OUT_T_DELTA_LSB               (0x0B)

#define MPL3115A2_WHO_AM_I                      (0x0C)

#define MPL3115A2_PT_DATA_CFG                   (0x13)
#define MPL3115A2_PT_DATA_CFG_TDEFE             (0x01)
#define MPL3115A2_PT_DATA_CFG_PDEFE             (0x02)
#define MPL3115A2_PT_DATA_CFG_DREM              (0x04)

#define MPL3115A2_CTRL_REG1                     (0x26)
#define MPL3115A2_CTRL_REG1_SBYB                (0x01)
#define MPL3115A2_CTRL_REG1_OST                 (0x02)
#define MPL3115A2_CTRL_REG1_RST                 (0x04)
#define MPL3115A2_CTRL_REG1_OS1                 (0x00)
#define MPL3115A2_CTRL_REG1_OS2                 (0x08)
#define MPL3115A2_CTRL_REG1_OS4                 (0x10)
#define MPL3115A2_CTRL_REG1_OS8                 (0x18)
#define MPL3115A2_CTRL_REG1_OS16                (0x20)
#define MPL3115A2_CTRL_REG1_OS32                (0x28)
#define MPL3115A2_CTRL_REG1_OS64                (0x30)
#define MPL3115A2_CTRL_REG1_OS128               (0x38)
#define MPL3115A2_CTRL_REG1_RAW                 (0x40)
#define MPL3115A2_CTRL_REG1_ALT                 (0x80)
#define MPL3115A2_CTRL_REG1_BAR                 (0x00)
#define MPL3115A2_CTRL_REG2                     (0x27)
#define MPL3115A2_CTRL_REG3                     (0x28)
#define MPL3115A2_CTRL_REG4                     (0x29)
#define MPL3115A2_CTRL_REG5                     (0x2A)

#define MPL3115A2_REGISTER_STARTCONVERSION      (0x12)


/**
 * @brief      Constructs the MPL3115A2 driver.
 */
MPL3115A2::MPL3115A2 ()
{
	device_mode = 0;
}

/**
 * @brief      I2C Read Execution for the MPL3115A2
 *
 * @param[in]  read_register  the address of the register to which we want to
 *                            read.
 *
 * @return     the byte value of the read register.
 */
uint8_t MPL3115A2::i2c_read( uint8_t read_register ) {
	Wire.beginTransmission( MPL3115A2_ADDRESS );
	Wire.write( read_register );
	Wire.endTransmission( false );

	Wire.requestFrom( (uint8_t) MPL3115A2_ADDRESS, (uint8_t) 1 );
	return Wire.read();
}

void MPL3115A2::i2c_write( uint8_t reg_addr, uint8_t value ) {
	Wire.beginTransmission( MPL3115A2_ADDRESS );
	Wire.write( reg_addr );
	Wire.write( value );
	Wire.endTransmission(false);
}

/**
 * @brief      Sets the measurement mode of the MPL3115A2.
 *
 * @param[in]  mode  the MPL3115A2 measurement mode: 3 valid options:
 *       - MPL3115A2_CTRL_REG1_RAW = 0x40
 *       - MPL3115A2_CTRL_REG1_ALT = 0x80
 *       - MPL3115A2_CTRL_REG1_BAR = 0x00
 */
void MPL3115A2::set_device_mode ( uint8_t mode ) 
{
	device_mode = mode;
}

/**
 * @brief      Initialize the MPL3115A2 Device
 *
 * @param[in]  altitude_mode  Init device to use altitude as its intial setup,
 *                            this can always be changed later.
 *
 * @return     true if the device has been initialized properly.
 */
bool MPL3115A2::init ( bool altitude_mode ) {
	bool init_success = false;
	Wire.begin();
	uint8_t who_am_i = i2c_read( MPL3115A2_WHO_AM_I );
	if (who_am_i == 0xC4) {
		init_success = true;
	}

	//set_device_mode( MPL3115A2_CTRL_REG1_ALT );
	i2c_write( MPL3115A2_CTRL_REG1,
	 MPL3115A2_CTRL_REG1_SBYB |
	 MPL3115A2_CTRL_REG1_OS128 |
	 MPL3115A2_CTRL_REG1_ALT);

	i2c_write( MPL3115A2_PT_DATA_CFG, 
			   MPL3115A2_PT_DATA_CFG_TDEFE | MPL3115A2_PT_DATA_CFG_PDEFE |
			   MPL3115A2_PT_DATA_CFG_DREM );

	return init_success;
}

/**
 * @brief      Get the MPL3115A2 Pressure Reading
 *
 * @param[out] pressure  the returned pressure result - valid if the function
 *                       returns true.
 *
 * @return     true if the pressure value is valid.
 * @return     false if the pressure reading is invalid, mode is incorrect or other
 *             failures.
 */
bool MPL3115A2::getPressure( uint32_t* pressure )
{
	bool valid_pressure = false;

	i2c_write(MPL3115A2_CTRL_REG1, 
			MPL3115A2_CTRL_REG1_SBYB |
			MPL3115A2_CTRL_REG1_OS128 |
			MPL3115A2_CTRL_REG1_BAR);

	uint8_t sta = 0;
	while (! (sta & MPL3115A2_REGISTER_STATUS_PDR)) {
		sta = i2c_read(MPL3115A2_REGISTER_STATUS);
		delay(10);
	}
	
	Wire.beginTransmission(MPL3115A2_ADDRESS); // start transmission to device 
	Wire.write(MPL3115A2_REGISTER_PRESSURE_MSB); 
	Wire.endTransmission(false); // end transmission

	Wire.requestFrom((uint8_t)MPL3115A2_ADDRESS, (uint8_t)3);// send data n-bytes read
	
	*pressure = Wire.read(); // receive DATA
	*pressure <<= 8;
	*pressure |= Wire.read(); // receive DATA
	*pressure <<= 8;
	*pressure |= Wire.read(); // receive DATA
	*pressure >>= 4;
	valid_pressure = true;

	return valid_pressure;
}

/**
 * @brief      Get the MPL3115A2 Pressure Reading
 *
 * @param[out] pressure  the returned pressure result - valid if the function
 *                       returns true.
 *
 * @return     true if the pressure value is valid.
 * @return     false if the pressure reading is invalid, mode is incorrect or other
 *             failures.
 */
bool MPL3115A2::getAltitude( uint32_t* altitude )
{
	bool valid_altitude = false;

	//if ( device_mode == MPL3115A2_CTRL_REG1_ALT ) {
		i2c_write( MPL3115A2_CTRL_REG1, 
	 			   MPL3115A2_CTRL_REG1_SBYB |
				   MPL3115A2_CTRL_REG1_OS128 |
	 			   MPL3115A2_CTRL_REG1_ALT);

		uint8_t sta = 0;
		while (! (sta & MPL3115A2_REGISTER_STATUS_PDR)) {
			sta = i2c_read(MPL3115A2_REGISTER_STATUS);
			delay(10);
		}
		
		Wire.beginTransmission(MPL3115A2_ADDRESS); // start transmission to device 
		Wire.write(MPL3115A2_REGISTER_PRESSURE_MSB); 
		Wire.endTransmission(false); // end transmission

		uint8_t bytes_rxd = Wire.requestFrom((uint8_t)MPL3115A2_ADDRESS, (uint8_t)3);// send data n-bytes read
		if ( 2 < bytes_rxd ) {
			*altitude = Wire.read(); // receive DATA
			*altitude <<= 8;
			*altitude |= Wire.read(); // receive DATA
			*altitude <<= 8;
			*altitude |= Wire.read(); // receive DATA
			*altitude >>= 4;

		  	if (*altitude & 0x80000) {
				*altitude |= 0xFFF00000;
		  	}
		  	//float altitude = altitude;
			//altitude /= 16.0;
			valid_altitude = true;
		}
	return valid_altitude;
}


float MPL3115A2::getPressure_Pa( void ) {
	uint32_t pressure;
	float f_press = -9999;

    if ( getPressure( &pressure ) ) {
        f_press = pressure;
        f_press /= 4.0;
    }

    return f_press;
}

float MPL3115A2::getPressure_InHg( void ) {
	return (getPressure_Pa()/3386.38);
}

float MPL3115A2::getTemperature() {
	uint16_t t_u16;
	uint8_t sta = 0;

	while (! (sta & MPL3115A2_REGISTER_STATUS_TDR)) {
		sta = i2c_read(MPL3115A2_REGISTER_STATUS);
		delay(10);
	}
	
	Wire.beginTransmission(MPL3115A2_ADDRESS); // start transmission to device 
	Wire.write(MPL3115A2_REGISTER_TEMP_MSB); 
	Wire.endTransmission(false); // end transmission
	float temp;
	uint8_t bytes_rxd = Wire.requestFrom((uint8_t)MPL3115A2_ADDRESS, (uint8_t)2);// send data n-bytes read
	
	if ( 1 < bytes_rxd) {
		t_u16 = Wire.read(); // receive DATA
		t_u16 <<= 8;
		t_u16 |= Wire.read(); // receive DATA
		t_u16 >>= 4;
		temp = t_u16;
		temp /= 16.0;
	}
	else {
		temp = -999;
	}
	
	return temp;
}

void MPL3115A2::setAltitude_Mode( void ) {
	set_device_mode( MPL3115A2_CTRL_REG1_ALT );
}

void MPL3115A2::setPressure_Mode( void ) {
	set_device_mode( MPL3115A2_CTRL_REG1_BAR );
}

/** @} end of addtogroup */