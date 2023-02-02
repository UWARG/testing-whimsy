#include <Airspeed.hpp>


Airspeed_MS4525DO::Airspeed_MS4525DO(I2C_HandleTypeDef* dev)
{
	I2C = dev;
	airspeed = 0;
	dif_pressure = 0;
	temperature = 0;
	dp_raw = 0; //raw pressure data
	dt_raw = 0; //raw temperature data
	strcpy((char*)msg, "NotStarted\r\n");
}

Airspeed_MS4525DO::~Airspeed_MS4525DO()
{

}

bool Airspeed_MS4525DO::init()
{
		uint8_t buf[4];
		buf[0] = 0x00;
		HAL_StatusTypeDef ret;
		ret = HAL_I2C_Master_Transmit(I2C, (uint16_t)MS4525D0_I2C_ADDR1 << 1, buf, 1, 50);
	    if ( ret != HAL_OK ) {
	      strcpy((char*)msg, "Error Tx\r\n");
	      return false;
	    } else {

	      for(int i = 0; i < 5000; i++);
	      // Read 4 bytes from the airspeed register
	      ret = HAL_I2C_Master_Receive(I2C, ((uint16_t)MS4525D0_I2C_ADDR1 << 1 )| 0x01, buf, 4, 50);
	      if ( ret != HAL_OK ) {
	        strcpy((char*)msg, "Error Rx\r\n");
	        return false;
	      } else {

	        //interpreting raw data
	    	  dp_raw = ((int16_t)buf[0] << 8) + (int16_t)buf[1];
	    	  dp_raw = 0x3FFF & dp_raw;

	    	  dt_raw = ((int16_t)buf[2] << 8) + (int16_t)buf[3];
	    	  dt_raw = (0xFFE0 & dt_raw) >> 5;

	    	  if (dp_raw  == 0x3FFF || dp_raw  == 0 || dt_raw  == 0x7FF || dt_raw == 0)
			  {
				  strcpy((char*)msg, "Error Dx\r\n");
				  return false;
			  }
	      }
	    }
	    calculate_pressure();
	    calculate_temperature();
	    calculate_airspeed();

	    return true;
}

void Airspeed_MS4525DO::calculate_pressure()
{
	const float P_max = 150;
	const float P_min = -P_max;
	// pound-force per square inch to pascal(newton per square meter)
	const float PSI_to_Pa = 6894.757f;

	//calculation can be different depend on the output type(A/B)
	float press_PSI  = -((dp_raw - 0.1f*16383) * (P_max-P_min)/(0.8f*16383) + P_min);
	dif_pressure  = press_PSI * PSI_to_Pa;
}

void Airspeed_MS4525DO::calculate_temperature()
{
	//temperature in Celsius
	temperature  = ((200.0f * dt_raw) / 2047) - 50;
}

void Airspeed_MS4525DO::calculate_airspeed()
{

	const float RHO = 1.225; // air density
	airspeed = sqrt((2*dif_pressure) / RHO); //km/hr
}


float Airspeed_MS4525DO::get_pressure() const
{
	sprintf((char*)msg, "%u Pre\r\n",((unsigned int)dif_pressure));
	return dif_pressure;
}

float Airspeed_MS4525DO::get_temperature() const
{
	sprintf((char*)msg, "%u Temp\r\n",((unsigned int)temperature));
	return temperature;
}
float Airspeed_MS4525DO::get_airspeed( ) const
{
	sprintf((char*)msg, "%u AirS\r\n",((unsigned int)airspeed));
	return airspeed;
}
