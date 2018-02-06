//------------------------------------------------------------------------------
/// @addtogroup htu21d_driver HUT21D Driver
///
/// @file drv_htu21d.cpp
///
/// @author     Joshua R. Talbot
///
/// @date       09-FEB17
///
/// @version    1.0 - initial release
///
/// @license
/// MIT License
/// 
/// Copyright (c) 2018 J. Talbot
/// 
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
///
///
//------------------------------------------------------------------------------
#include "Arduino.h"
#include <stdint.h>
#include <stdbool.h>

class DRV_HTU21D {
    public:
        DRV_HTU21D();
        bool init( void );
        void reset(void);
        void setConfig( void );
        uint16_t getConfig(void);
        float getTemp_C(void);
        float getTemp_F(void);
        float getHumidity(void);
        void setResolution( uint8_t );
        void setHeater( bool );
    private:
        bool read_HUT_Config(void);
        uint8_t check_crc8(uint16_t, uint8_t);
        uint8_t user_register;
        bool config_changed;
};