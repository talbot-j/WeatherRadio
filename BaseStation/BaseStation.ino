//------------------------------------------------------------------------------
/// @addtogroup weather_station Mini Backyard Weather Station
///
/// @file WeatherStation.ino
///
/// @author     Joshua R. Talbot
///
/// @date       02-FEB-18
///
/// @version    1.0 - initial release
///
/// @license
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
/// @brief      The start of a backyard weather station using the Sparkfun
///             weather board.
///
///
///  
/// Information that Wunderground wants to see...
/// 
/// winddir - [0-360 instantaneous wind direction]
/// windspeedmph - [mph instantaneous wind speed]
/// windgustmph - [mph current wind gust, using software specific time period]
/// windgustdir - [0-360 using software specific time period]
/// windspdmph_avg2m  - [mph 2 minute average wind speed mph]
/// winddir_avg2m - [0-360 2 minute average wind direction]
/// windgustmph_10m - [mph past 10 minutes wind gust mph ]
/// windgustdir_10m - [0-360 past 10 minutes wind gust direction]
/// humidity - [% outdoor humidity 0-100%]
/// dewptf- [F outdoor dewpoint F]
/// tempf - [F outdoor temperature] 
/// rainin - [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
/// dailyrainin - [rain inches so far today in local time]
/// baromin - [barometric pressure inches]
/// 
/// https://weatherstation.wunderground.com/weatherstation/updateweatherstation.php?ID=KCASANFR5&PASSWORD=XXXXXX&dateutc=2000-01-01+10%3A32%3A35&winddir=230&windspeedmph=12&windgustmph=12&tempf=70&rainin=0&baromin=29.1&dewptf=68.2&humidity=90&weather=&clouds=&softwaretype=vws%20versionxx&action=updateraw
/// 

#include <stdint.h>
#include <string.h>
#include <SPI.h>
#include <RFM69.h>

#define GATEWAYID   	1			/* ID of the data gateway/server */
#define RADIO_NODEID 	1			/* ID if weather station */
#define RADIO_NETID		100			/* Network ID */
#define RADIO_FREQ		RF69_915MHZ  /* See RFM69.h for other freq options*/
#define RADIO_CRYPT_KEY "Wh@t5MyN@m3"
#define SERIAL_BAUD 	57600

/* Status flags */
#define RADIO_OK			(1<<0)
#define HUMIDITY_OK			(1<<1)
#define BARO_OK				(1<<2)
#define WSTATION_OK			(1<<3)
#define SENSOR_ERR			(1<<4)
#define RADIO_TIMEOUTS		(1<<5)

typedef struct PAYLOAD_DATA_STRUCT {
	uint16_t node_id;
	uint16_t status_flags;
	uint32_t windspeedmph;
	int16_t winddir_x;
	int16_t winddir_y;
	uint16_t windgustmph;
	uint16_t windspdmph_avg2m;
	int16_t winddir_avg2m_x;
	int16_t winddir_avg2m_y;
	uint16_t humidity_x10;
	int16_t temperature_F_x10;
	uint16_t pressure_inHg_x10;
	uint16_t acc_rain_1m;
	uint16_t acc_rain_1hr;
	uint16_t acc_rain_24hr;
	uint16_t uv_light;
	uint16_t light_level;
	uint16_t pool_temp_F_x10;
	unsigned long uptime;
} WEATHER_RADIO_PAYLOAD_T;


/* Declare a instance of the Radio
 * (part of the the Low Power Lab Moteino) */
RFM69 radio;

/* Define the WeatherData Object - used to hold and transfer 
 * current weather data and station information back to the 
 * base station */
WEATHER_RADIO_PAYLOAD_T weather_data;

/* Define some timers required to handle the weather station
function calls in a "timely" fashion */
long timer_1s_millis;	/* 1 second timer accumulator */
long timer_5s_millis;	/* 5 second timer accumulator */
long timer_60s_millis;	/* 1 minute timer accumulator */

/* Timer presets (set point) for the 1, 5 and 60 second timers. */
const long timer_1s_preset = 1000;
const long timer_5s_preset = 5000;
const long timer_60s_preset = 60000;


/**
 * @brief      Sets or Clears a particular flag in the Weather Radio Data
 *             payload
 *
 * @param      data   a pointer to the weather data
 * @param[in]  flag   The flag to set or clear
 * @param[in]  state  The action - set or clear the flag.
 */
void WDAT_set_status_register( WEATHER_RADIO_PAYLOAD_T* data, const uint16_t flag, const bool state ) {
	if (state) {
		data->status_flags |= flag;
	}
	else {
		data->status_flags &= ~flag;
	}
}

/**
 * @brief      Timer Reset Function - resets accumulator back to current time.
 *
 * @param[out] timer_value  The timer value to be written
 */
void timer_reset( long *timer_value ) {
	*timer_value = millis();
}

/**
 * @brief      Determines if the timer has completed (done).
 *
 * @param      timer_value  The timer accumulator (current value)
 * @param[in]  preset       The preset (or set point)
 *
 * @return     True if timer done, False otherwise.
 */
bool is_timer_done( long *timer_value, long preset) {
	long current_time = millis();
	if ( (current_time - *timer_value) > preset) {
		*timer_value = current_time;
		return true;
	}
	return false;
}

bool init_radio ( void ) {
	radio.initialize(RADIO_FREQ, RADIO_NODEID, RADIO_NETID);
	radio.encrypt(RADIO_CRYPT_KEY);
	radio.promiscuous(false); /* disable sniffing of all the packets over the air */
	
	uint32_t freq;
	if ( RADIO_FREQ == RF69_315MHZ ) {
		freq = 315;
	}
	else if ( RADIO_FREQ == RF69_433MHZ ) {
		freq = 433;
	}
	else if ( RADIO_FREQ == RF69_868MHZ ) {
		freq = 868;
	}
	else {
		freq = 915;
	}

	char buff[80];
	sprintf(buff, "Listening at %u MHz.", freq );
	Serial.println(buff);

	return true;
}

/**
 * @brief      Critical Error has Occurred so Lockup the processor gracefully
 *             and flash a LED (TBD) to indicate a critical error with the radio
 *             has occurred.  If the radio is up, all other "bad" status
 *             information can be sent to the base station.
 *
 * @param[in]  flag  The error flag that is causing the lockup condition.
 */
void Error_LockUp( uint16_t flag ) {
	while(1) {
		/* Use this to Flash Lockup Status on a LED T.B.D. */
	}
}

void Serial_Print_SenderID ( void ) {
	Serial.print('[');
	Serial.print(radio.SENDERID, DEC);
	Serial.print("] ");
}

void Serial_Print_RSSI( void ){
	Serial.print(" [RX_RSSI:");
	Serial.print(radio.readRSSI());
	Serial.print("]");
}


void Serial_Print_Invalid_DataRXD( void ){
	Serial.println("Invalid payload received, not matching Payload struct!");
	Serial.println("Payload size is");
	Serial.print(radio.DATALEN);
}


void Serial_Print_Weather_Data( WEATHER_RADIO_PAYLOAD_T *data ){
	Serial.print("NodeID=");
	Serial.println(data->node_id);

	Serial.print("Ave Temp (F)=");
	Serial.println(data->temperature_F_x10/10.0) ;
}

#if 0
void load_current_radio_data ( void ) {
	weather_data.node_id = RADIO_NODEID; /* uint16_t node_id; */  
	//weather_data.status_flags;		/* uint16_t status_flags; */  
	
	wStation.get_last_a5s_wind( 
		&(weather_data.winddir_x),
		&(weather_data.winddir_y),
		&(weather_data.windspeedmph) ); 
	weather_data.windgustmph = 3;		/* uint16_t windgustmph; */  
	weather_data.windspdmph_avg2m = 4;	/* uint16_t windspdmph_avg2m; */  
	weather_data.winddir_avg2m_x = 5;	/* int16_t winddir_avg2m_x; */  
	weather_data.winddir_avg2m_y= 6;	/* int16_t winddir_avg2m_y; */  
	weather_data.humidity_x10 = (uint16_t) (hum_sensor.getHumidity() * 10) ;		/* uint16_t humidity_x10; */  
	WSD_get_average_temperature_F( &(weather_data.temperature_F_x10) ); /* int16_t temperature_F_x10; */  
	weather_data.pressure_inHg_x10 = 0;	/* uint16_t pressure_inHg_x10; */  
	weather_data.acc_rain_1m = 999;		/* uint16_t acc_rain_1m; */  
	weather_data.acc_rain_1hr = 999;		/* uint16_t acc_rain_1hr; */  
	weather_data.acc_rain_24hr = 999;		/* uint16_t acc_rain_24hr; */  
	weather_data.uv_light = 0; 			/* uint16_t uv_light; */  
	weather_data.light_level = 0; 		/* uint16_t light_level; */  
	weather_data.pool_temp_F_x10 = 0; 	/* uint16_t pool_temp_F_x10; */  
	weather_data.uptime = millis();	 		/* unsigned long uptime;	 */  
}
#endif

/**
 * @brief      Setup function - the built in Arduino initialization function
 */
void setup()
{
	Serial.begin(SERIAL_BAUD);
	
	bool radio_status = init_radio();
	
	timer_reset(&timer_1s_millis);
	timer_reset(&timer_5s_millis);
	timer_reset(&timer_60s_millis);
}

void send_radio_data( const void *data, size_t data_len ) {
	if ( radio.sendWithRetry( GATEWAYID, data, data_len ) ) {
    	Serial.println("Sent Data!");
	}
    else {
    	Serial.println("Error Sending Data.");
    }
}

void loop() {

	if ( radio.receiveDone() ){
		Serial_Print_SenderID();
		Serial_Print_RSSI();
		if (radio.DATALEN != sizeof(WEATHER_RADIO_PAYLOAD_T)) {
			Serial_Print_Invalid_DataRXD();
		}
		else {
			if (radio.ACKRequested()) {
				//byte theNodeID = radio.SENDERID;
				radio.sendACK();
				Serial.println(" - ACK sent.");
			}
			WEATHER_RADIO_PAYLOAD_T *w_data;
			w_data = (WEATHER_RADIO_PAYLOAD_T*) &(radio.DATA); //assume radio.DATA actually contains our struct and not something else
			Serial_Print_Weather_Data( w_data );

		}
	}

	#if 0
	if ( is_timer_done( &timer_5s_millis, timer_5s_preset ) ) {
		//Serial.println("\n---------------\n");
		//print_wind_data();
		//print_temperatures();
		//Serial.print("Light Level: ");Serial.println(get_light_level());
		load_current_radio_data();
		send_radio_data( &weather_data, sizeof(weather_data) );
	}
	

	if ( is_timer_done( &timer_1s_millis, timer_1s_preset ) ) {
		wStation.wind_calcs_per_second();
	}

	if ( is_timer_done( &timer_60s_millis, timer_60s_preset ) ) {
		wStation.rain_calcs_per_minute();
		//load_current_radio_data();
		//send_radio_data( &weather_data, sizeof(weather_data) );
	}
	#endif
}
