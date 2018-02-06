/*-----------------------------------------------*/
/** @addtogroup WSA80422_device Argent Weather Sensor Assembly 
 * @{
 *
 * @file WSA80422.cpp
 *
 * @author     Joshua R. Talbot
 *
 * @date       12-FEB-2017
 */

#include "WSA80422.h"
extern void windIRQ( void );
extern void rainIRQ( void );

const int16_t wind_vector_ary[] =
	{
		(int16_t) WDIR_N,   0,     1000, /* Offset:  0 */
		(int16_t) WDIR_NNW, 383,    924, /* Offset:  3 */
		(int16_t) WDIR_NW,  707,    707, /* Offset:  6 */
		(int16_t) WDIR_WNW, 924,    383, /* Offset:  9 */
		(int16_t) WDIR_W,   1000,     0, /* Offset: 12 */
		(int16_t) WDIR_WSW, 924,   -383, /* Offset: 15 */
		(int16_t) WDIR_SW,  707,   -707, /* Offset: 18 */
		(int16_t) WDIR_SSW, 382,   -924, /* Offset: 21 */
		(int16_t) WDIR_S,   0,    -1000, /* Offset: 24 */
		(int16_t) WDIR_SSE, -383,  -924, /* Offset: 27 */
		(int16_t) WDIR_SE,  -707,  -707, /* Offset: 30 */
		(int16_t) WDIR_ESE, -924,  -383, /* Offset: 33 */
		(int16_t) WDIR_E,   -1000,    0, /* Offset: 36 */
		(int16_t) WDIR_ENE, -924,   383, /* Offset: 39 */
		(int16_t) WDIR_NE,  -707,   707, /* Offset: 42 */
		(int16_t) WDIR_NNE, -383,   924, /* Offset: 45 */
		(int16_t) WDIR_ERR,    0,	  0, /* Offset: 48 */
	};


WSA80422::WSA80422() {
	rain_fall_acc = 0;
	wind_count = 0;
	winddir_pin = A0;
	time_of_last_wind_read = 0;
	time_of_last_rain_read = 0;
}

bool WSA80422::init ( uint8_t rain_pin, uint8_t wspd_pin, uint8_t wdir_pin ) {
	bool config_success = true;

	uint8_t wind_input_debounce_period = 10;
	uint8_t rain_input_debounce_period = 10;
	
	pinMode(wspd_pin, INPUT_PULLUP);
	pinMode(rain_pin, INPUT_PULLUP);
	
	if ( wspd_pin == 2 ) {
		attachInterrupt(0, windIRQ, FALLING);
	}
	else if ( wspd_pin == 3 ) {
		attachInterrupt(1, windIRQ, FALLING);
	}
	else {
		config_success = false;
	}
	
	if ( rain_pin == 2 ) {
		attachInterrupt(0, rainIRQ, FALLING);
	}
	else if ( rain_pin == 3 ) {
		attachInterrupt(1, rainIRQ, FALLING);
	}
	else {
		config_success = false;
	}
	rain_fall_acc = 0;
	wind_count = 0;
	winddir_pin = wdir_pin;

	time_of_last_rain_read = millis();
	time_of_last_wind_read = millis();

	interrupts();

	return config_success;
}

void WSA80422::wind_reset_arrays ( void ) {
	uint8_t i;

	for( i=0; i<5; i++) {
		w_dir_5s_x[i] = 0;
		w_dir_5s_y[i] = 0;	
	}
	
	for( i=0; i<24; i++) {
		w_dir_2m_x[i] = 0;
		w_dir_2m_y[i] = 0;	
	}
}

WINDDIR_T WSA80422::getWindDir() {

	uint16_t adc = getWindDirRaw();
	WINDDIR_T direction;

	// Direction Degrees: 113
	if (adc < 380)
		{ direction = WDIR_ESE;  }      
	// Direction Degrees: 68
	else if (adc < 393) 
		{ direction = WDIR_ENE; }  
	// Direction Degrees: 90
	else if (adc < 414) 
		{ direction = WDIR_E;  }   
	// Direction Degrees: 158
	else if (adc < 456) 
		{ direction = WDIR_SSE;  } 
	// Direction Degrees: 135
	else if (adc < 508) 
		{ direction = WDIR_SE;  }  
	// Direction Degrees: 203
	else if (adc < 551) 
		{ direction = WDIR_SSW;  } 
	// Direction Degrees: 180
	else if (adc < 615) 
		{ direction = WDIR_S; }    
	// Direction Degrees: 23
	else if (adc < 680) 
		{ direction = WDIR_NNE; }  
	// Direction Degrees: 45
	else if (adc < 746) 
		{ direction = WDIR_NE; }   
	// Direction Degrees: 248
	else if (adc < 801) 
		{ direction = WDIR_WSW;  } 
	// Direction Degrees: 225
	else if (adc < 833) 
		{ direction = WDIR_SW;  }   
	// Direction Degrees: 338
	else if (adc < 878) 
		{ direction = WDIR_NNW;  } 
	// Direction Degrees: 0
	else if (adc < 913) 
		{ direction = WDIR_N; }    
	// Direction Degrees: 293
	else if (adc < 940) 
		{ direction = WDIR_WNW;  } 
	// Direction Degrees: 315
	else if (adc < 967) 
		{ direction = WDIR_NW;  }  
	// Direction Degrees: 270
	else if (adc < 990) 
		{ direction = WDIR_W;  }
	else {
		direction = WDIR_ERR;
	}   
	return (direction);
}

uint16_t WSA80422::getWindDirRaw( void ) {
	return analogRead( winddir_pin );
}


uint16_t WSA80422::getWindAcc( void ) {
	return wind_count;
}

void WSA80422::resetWindAcc( void ) {
	wind_count = 0;
}

uint16_t WSA80422::getRainFall( void ) {
	return rain_fall_acc;
}

void WSA80422::resetRainFallAcc( void ) {
	rain_fall_acc = 0;
}

void WSA80422::get_last_a5s_wind( int16_t *x, int16_t *y, uint32_t *spd) {
	uint8_t previous_idx;
    if ( idx2m == 0 ) {
    	previous_idx = 23;
    }
    else {
    	previous_idx = idx2m - 1;
    }

    *x = w_dir_2m_x[previous_idx];
    *y = w_dir_2m_y[previous_idx];
    *spd = w_spd_2m[previous_idx];
}

void WSA80422::get_last_a1m_rain( uint16_t *rain ) {
	uint8_t previous_idx;
    if ( rf_idx1m == 0 ) {
    	previous_idx = 59;
    }
    else {
    	previous_idx = rf_idx1m - 1;
    }

    *rain = acc_rain_1m[previous_idx];
}

bool WSA80422::init_light_sensor( uint8_t light_pin, uint8_t ref_pin ) {
	LIGHT_PIN = light_pin;
	REF_3V3_PIN = ref_pin;
	pinMode(LIGHT_PIN, INPUT);
	pinMode(REF_3V3_PIN, INPUT);
}

//Returns the voltage of the light sensor based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
float WSA80422::get_light_level()
{
	float operatingVoltage = analogRead(REF_3V3_PIN);

	float lightSensor = analogRead(LIGHT_PIN);

	operatingVoltage = 3.3 / operatingVoltage; //The reference voltage is 3.3V

	lightSensor = operatingVoltage * lightSensor;

	return(lightSensor);
}

void WSA80422::get_last_a1hr_24hr_rain( uint16_t *rain_1hr, uint16_t *rain_day ) {
	uint8_t previous_idx;
    if ( rf_idx1hr == 0 ) {
    	previous_idx = 23;
    }
    else {
    	previous_idx = rf_idx1hr - 1;
    }

    *rain_1hr = acc_rain_1hr[previous_idx];

    uint8_t i;
    uint16_t acc_daily_rain = 0;
    for ( i=0; i<24; i++) {
    	acc_daily_rain += acc_rain_1hr[i];
    }

    *rain_day = acc_daily_rain;
}


void WSA80422::get_a2m_wind( int16_t *x, int16_t *y, uint32_t *spd) {
	*x = 0;
    *y = 0;
    *spd = 0;
}

void WSA80422::rain_calcs_per_minute ( void ) {
	
	noInterrupts();
	acc_rain_1m[rf_idx1m] = getRainFall();
	resetRainFallAcc();
	interrupts();

	rf_idx1m += 1;
	if ( rf_idx1m == 60 ) {
		uint8_t i;
		uint16_t total_rf = 0;
		for( i=0; i<60; i++ ){
			total_rf += acc_rain_1m[i];
			acc_rain_1m[i] = 0;	
		}
		rf_idx1hr += 1;
		if ( 24 == rf_idx1hr ) {
			rf_idx1hr = 0;
		}
	}
}

void WSA80422::wind_calcs_per_second ( void ) {
	int16_t x, y;
	WINDDIR wind_dir = getWindDir();
	
	noInterrupts();
	uint32_t wind_spd_pls = getWindAcc();
	resetWindAcc();
	interrupts();
	uint32_t speed = wind_spd_pls * 1492;
    x = wind_vector_ary[3*wind_dir+1];
	y = wind_vector_ary[3*wind_dir+2];

	w_dir_5s_x[idx5s] = x;
	w_dir_5s_y[idx5s] = y;
	w_spd_5s[idx5s] = speed;
	idx5s += 1;
	if ( idx5s == 5 ) {
		int16_t avg_x;
		int16_t avg_y;
		uint32_t avg_spd;
		uint8_t i;
		for ( i = 0; i<5; i++ ) {
			avg_y += w_dir_5s_y[i];
			avg_x += w_dir_5s_x[i];
			avg_spd += w_spd_5s[i];
		}
		avg_y = avg_y/5;
		avg_x = avg_x/5;
		avg_spd = avg_spd/5;
		w_dir_2m_x[idx2m] = avg_x;
		w_dir_2m_y[idx2m] = avg_y;
		w_spd_2m[idx2m] = avg_spd;
		idx2m += 1;
		if (idx2m == 24) {
			idx2m = 0;
		}
		//Serial.print("Wind Avgs: x=");Serial.print(avg_x);Serial.print(" y=");Serial.println(avg_y);
		idx5s = 0;
	}
}

void WSA80422::rainIRQ_CB( void ) {
	if ( (millis() - time_of_last_rain_read) > rain_input_debounce_period) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
	{
		time_of_last_rain_read = millis(); //Grab the current time
		rain_fall_acc += 11; //There is 11 thousands of an inch of rain for every tip of the bucket.
	}
}

void WSA80422::windIRQ_CB( void ) {
	if ( (millis() - time_of_last_wind_read) > wind_input_debounce_period) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
	{
		time_of_last_wind_read = millis(); //Grab the current time
		wind_count++; //There is 1.492MPH for each click per second.
	}
}