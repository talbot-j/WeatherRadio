/*-----------------------------------------------*/
/** @addtogroup WSA80422_device Argent Weather Sensor Assembly 
 * @{
 *
 * @file WSA80422.h
 *
 * @author     Joshua R. Talbot
 *
 * @date       12-FEB-2017
 */

/** @details    <b>Anemometer</b>
 *
 *             The cup-type anemometer measures wind speed by closing a contact
 *             as a magnet moves past a switch. A wind speed of 1.492 MPH (2.4
 *             km/h) causes the switch to close once per second.
 *
 *             <b>Rain Gauge</b>
 *
 *             The rain gauge is a self-emptying tipping bucket type. Each
 *             0.011” (0.2794 mm) of rain causes one momentary contact closure.
 *             
 *             If stored as 1000ths of inches it appears uint16 size integers will be fine...
 *             Most in one minute: 1.5 in (Barot, Guadeloupe, 26 November 1970)
 * 			   Most in one hour: 12.0 in 42 minutes. Holt, Missouri, 22 June 1947.
 * 			   Most in 12 hours (1/2 day): 1,144 mm (45.0 in); Cilaos, Réunion, 8 January 1966, during Tropical Cyclone Denise.[82]
 * 			   Most in 24 hours (1 day): 1,825 mm (71.9 in); Cilaos, Réunion, 7–8 January 1966, during Tropical Cyclone Denise.[82]
 *
 *             <b>Wind Vane</b>
 *
 *             The wind vane is the most complicated of the three sensors. It
 *             has eight switches, each connected to a different resistor. The
 *             vane’s magnet may close two switches at once, allowing up to 16
 *             different positions to be indicated. An external resistor can be
 *             used to form a voltage divider, producing a voltage output that
 *             can be measured with an analog to digital converter, as shown
 *             below. The switch and resistor arrangement is shown in the
 *             diagram to the right. Resistance values for all 16 possible
 *             positions are given in the table. Resistance values for positions
 *             between those shown in the diagram are the result of two adjacent
 *             resistors connected in parallel when the vane’s magnet activates
 *             two switches simultaneously.
 *             
 *  |------------------------------------------|
 * 	| Direction |  Resistance |  Voltage	   |
 * 	| (Degrees) |  (Ohms)     |  (V=5v, R=10k) |
 *	|-----------|-------------|----------------|
 *	|    0.0	|  33k 		  |    3.84 V      |
 *	|   22.5 	|  6.57k 	  |    1.98 V      |
 *	|   45.0   	|  8.2k    	  |    2.25 V      |
 *	|   67.5  	|  891     	  |    0.41 V      |
 *	|   90.0   	|  1k      	  |    0.45 V      |
 *	|  112.5 	|  688     	  |    0.32 V      |
 *	|  135.0   	|  2.2k    	  |    0.90 V      |
 *	|  157.5 	|  1.41k   	  |    0.62 V      |
 *	|  180.0   	|  3.9k    	  |    1.40 V      |
 *	|  202.5 	|  3.14k   	  |    1.19 V      |
 *	|  225.0   	|  16k     	  |    3.08 V      |
 *	|  247.5 	|  14.12k  	  |    2.93 V      |
 *	|  270.0   	|  120k    	  |    4.62 V      |
 *	|  292.5 	|  42.12k  	  |    4.04 V      |
 *	|  315.0   	|  64.9k   	  |    4.78 V      |
 *	|  337.5 	|  21.88k  	  |    3.43 V      |
 *	|------------------------------------------|
 */

#ifndef WSA80422_H
#define WSA80422_H

#include <stdint.h>
#include <stdbool.h>
#include "Arduino.h"

typedef enum WINDDIR
{
	WDIR_N,
	WDIR_NNW,
	WDIR_NW,
	WDIR_WNW,
	WDIR_W,
	WDIR_WSW,
	WDIR_SW,
	WDIR_SSW,
	WDIR_S,
	WDIR_SSE,
	WDIR_SE,
	WDIR_ESE,
	WDIR_E,
	WDIR_ENE,
	WDIR_NE,
	WDIR_NNE,
	WDIR_ERR
} WINDDIR_T;

/* @brief      Rain Fall Report Type */
typedef enum RAIN_FALL_REPORT_PERIOD
{
	RF_LAST_HR,
	RF_LAST_24,
	RF_DAY
} RF_PERIOD_T;

class WSA80422 {
public:
	WSA80422();
	bool init ( uint8_t rain_pin, uint8_t wspd_pin, uint8_t wdir_pin );
	bool init_light_sensor( uint8_t light_pin, uint8_t ref_pin );
	WINDDIR_T getWindDir();
	uint16_t getWindDirRaw();
	uint16_t getWindAcc();
	void resetWindAcc( void );
	uint16_t getRainFall( void );
	void resetRainFallAcc( void );
	void rain_calcs_per_minute( void );
	void rainIRQ_CB( void );
	void windIRQ_CB( void );
	void wind_reset_arrays( void );
	void wind_calcs_per_second( void );
	void get_last_a5s_wind( int16_t *x, int16_t *y, uint32_t *spd);
	void get_a2m_wind( int16_t *x, int16_t *y, uint32_t *spd);
	void get_last_a1hr_24hr_rain( uint16_t *rain_1hr, uint16_t *rain_day );
	void get_last_a1m_rain( uint16_t *rain );
	float get_light_level( void );
private:
	uint16_t rain_fall_acc;
	uint16_t acc_rain_1m[60];
	uint16_t acc_rain_1hr[24];
	uint16_t wind_count;
	uint8_t winddir_pin;
	uint32_t time_of_last_wind_read;
	uint32_t time_of_last_rain_read;
	uint8_t wind_input_debounce_period;
	uint8_t rain_input_debounce_period;
	int16_t w_dir_5s_x[5];
	int16_t w_dir_5s_y[5];
	int16_t w_dir_2m_x[24];
	int16_t w_dir_2m_y[24];
	uint32_t w_spd_5s[5];
	uint32_t w_spd_2m[24];
	uint8_t idx5s;
	uint8_t idx2m;
	uint8_t rf_idx1m;
	uint8_t rf_idx1hr;
	uint8_t LIGHT_PIN;
	uint8_t REF_3V3_PIN;
};
#endif