/*
 * CPS.hpp
 *
 *  Created on: Jan 8, 2023
 *      Author: Hardy
 */

#ifndef INC_GPS_HPP_
#define INC_GPS_HPP_

#include <NMEAparse.h>
#include <stm32l5xx.h>
#include "main.h"



#define RAW_DATA_LENGTH 1000

class NEO_GPS
{
	private:
	char GGA[100];
	char RMC[100];
	GPSSTRUCT gpsData;
	UART_HandleTypeDef *UART;

	/*
	 * string is the key word of the protocol we are searching
	 * container is the output container
	 * length is the length of the key word
	 */
	bool get_sentense(const char* string, char* container, int length);

	public:

	uint8_t rx_raw[RAW_DATA_LENGTH];

    NEO_GPS(UART_HandleTypeDef* dev);
    ~NEO_GPS();

	bool initGPS();

	bool refreshGPS();

	UART_HandleTypeDef* get_uart_handler();
	LOCATION get_location();
	DATE get_date();
	TIME get_time();
	float get_speed();
	float get_course();
	int get_number_of_sat();
};



#endif /* INC_GPS_HPP_ */
