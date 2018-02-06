/*-----------------------------------------------*/
/** @addtogroup mpl3115a2_device MPL3115A2 Barometric/Pressure Sensor
 * @{
 *
 * @file MPL3115A2.h
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
 */

#ifndef MPL3115A2_H
#define MPL3115A2_H

//#include <cstdint>
//#include <cstdbool>

#define MPL3115A2_CTRL_REG1_RAW                 (0x40)
#define MPL3115A2_CTRL_REG1_ALT                 (0x80)
#define MPL3115A2_CTRL_REG1_BAR                 (0x00)

#if 1
class MPL3115A2 {
public:
	MPL3115A2 ();
	bool init ( bool );
	bool getPressure( uint32_t* pressure );
	bool getAltitude( uint32_t* altitude );
	void setAltitude_Mode( void );
	void setPressure_Mode( void );
	float getTemperature( void );
	float getFloatPressure( void );
	float getPressure_InHg( void );
	float getPressure_Pa( void );
private:
	uint8_t i2c_read( uint8_t read_register );
	void i2c_write( uint8_t reg_addr, uint8_t value );
	void set_device_mode ( uint8_t mode );
	uint8_t device_mode;
};
#endif
#endif

/** @} end of addtogroup */