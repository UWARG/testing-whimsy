#ifndef AIRSPEED_H_
#define AIRSPEED_H_

#include <math.h>
#include <string.h>
#include <stm32l5xx.h>
#include <stdio.h>

#define MS4525D0_I2C_ADDR1 0x28
#define MS4525D0_I2C_ADDR2 0x36
#define MS4525D0_I2C_ADDR3 0x46
//#define P_MAX_TYPEA 0x399A
//#define P_MIN_TYPEA 0x0666
//#define P_MAX_TYPEB 0x3CCB
//#define P_MIN_TYPEB 0x0333
//#define psi_range 30

class Airspeed_MS4525DO
{
	private:
		I2C_HandleTypeDef* I2C;
		float airspeed;
		float dif_pressure;
		float temperature;
		int16_t dp_raw; //raw pressure data
		int16_t dt_raw; //raw temperature data
		uint8_t msg[12];

		void calculate_pressure();
		void calculate_temperature();
		void calculate_airspeed();

	public:
		/*
		 * Default constructor of this object need the I2C handler
		 */
	    Airspeed_MS4525DO(I2C_HandleTypeDef* dev);
	    ~Airspeed_MS4525DO();
	    /*
	     * init function has to be called before fetching any
	     * data from sensor
	     */
	    bool init();
	    float get_pressure() const;
	    float get_temperature() const;
	    float get_airspeed( ) const;
};


#endif
