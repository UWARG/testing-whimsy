/*
 * GPS.cpp
 *
 *  Created on: Jan 8, 2023
 *      Author: Hardy
 */

/*
* configuration for the uart
* RCC - crystal/ceramic resonator on
* DMA - UART-RX circular mode on
*/

#include "GPS.hpp"
#include "string.h"



NEO_GPS::NEO_GPS(UART_HandleTypeDef* dev)
{
	UART = dev;
	HAL_UART_Receive_DMA(UART, rx_raw, RAW_DATA_LENGTH);
}

UART_HandleTypeDef* NEO_GPS::get_uart_handler() {
	return UART;
}

NEO_GPS::~NEO_GPS()
{

}

/*
 * private function that get the GGA or RMC sentence from raw data
 * then it can be sent for parsing
 */
bool NEO_GPS::get_sentense(const char* string, char* container, int length)
{
	int counter = 0;
	int len = length;
	bool new_sentense = false;
	while(counter < RAW_DATA_LENGTH && !new_sentense)
	{
		if(rx_raw[counter] == string[0])
		{
			int i = 1;
			bool same = true;
			while(i < len && same)
			{
				if(rx_raw[counter + i] != string[i])
					same = false;
				i++;
			}
			if(same)
			{
				while(rx_raw[counter + i] != '*' && counter + i < RAW_DATA_LENGTH)
				{
					container[i - len] = rx_raw[counter + i];
					i++;
				}
				//new_sentense = true;
				return true;
			}

		}

		counter++;
	}

	return false;
}

bool NEO_GPS::refreshGPS()
{

	const char GGAs[3] = {'G', 'G', 'A'};
	const char RMCs[3] = {'R', 'M', 'C'};

	if(get_sentense(GGAs, GGA, 3))
	{
		if(decodeGGA(GGA, &gpsData.ggastruct) != 0)
			return false;

	}
	else
	{
		return false;
	}

	if(get_sentense(RMCs, RMC, 3))
	{
		if(decodeRMC(RMC, &gpsData.rmcstruct) != 0)
			return false;
	}
	else
	{
		return false;
	}


	return true;
}

LOCATION NEO_GPS::get_location()
{
	return gpsData.ggastruct.lcation;
}
DATE NEO_GPS::get_date()
{
	return gpsData.rmcstruct.date;
}
TIME NEO_GPS::get_time()
{
	return gpsData.ggastruct.tim;
}
float NEO_GPS::get_speed()
{
	return gpsData.rmcstruct.speed;
}

float NEO_GPS::get_course()
{
	return gpsData.rmcstruct.course;
}
int NEO_GPS::get_number_of_sat()
{
	return gpsData.ggastruct.numofsat;
}






