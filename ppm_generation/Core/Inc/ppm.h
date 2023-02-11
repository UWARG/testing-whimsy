#include <stdint.h>

#define HIGH_PERC 1.0f
#define MID_PERC 0.5f
#define LOW_PERC 0.0f


#ifndef PPM_SIG

	/*The following are the important constant for the ppm generation process*/
	#define CORRECTION 1.0f /*this correction is used in psc and arr calculation*/

	static const float PULSE_WIDTH = 310.0f;//the time in us for high output
	static const float MIN_RESET_PULSE = 3000.0f;//the min length for the last sync; the end of ppm
	static const float MIN_PULSE_WIDTH = 700.0f;//the singal information min
	static const float MAX_PULSE_WIDTH = 1670.0f;//the signal info max
	static const float BASE_FREQUENCY = 48000000.0f;//the clock frequency for l552
	static const float SEC_TO_MICROSEC = 1000000.0f;//conversion factor for us to sec
	static const float DOWN_INTERVAL = MAX_PULSE_WIDTH - MIN_PULSE_WIDTH;//for percentage calc
	static const uint8_t MAX_CHANNEL = 16;//the max available for the driver
	static float counter, counter_to_microsec;//the first if the total number of counter will have
	//the second is the factor to convert the counter to microsec

	/*please configure this*/
	#define PSC_VALUE 14U/*prescalar used for the system (to determine the operating frequency*/
	#define channel_used 8//note that the max is 16 (to indicate the amount of channel used
	#define channel_reserved channel_used + 1//the last one is the reset pulse
	/*the user input for each channel of input*/
	static const float user_input[channel_used] =
	{HIGH_PERC,MID_PERC,LOW_PERC,HIGH_PERC,MID_PERC,LOW_PERC,HIGH_PERC,MID_PERC};
#endif /*PPM_SIG*/



/*__init__ initial all the variable that will be used
 * value need to be configured before hand is:
 * prescalar for clock
 * number of channel needed
 * all channel percentage input
 * base frequency
 */
void __init__(void);

/*get_ccr calculate for fetech the ccr value stored in it
 */
uint32_t get_ccr(void);

/*Microsecs_to_counter convert a time interval measured in microsecs to counter for ccr and arr
 * arg1: time_length: how long the interval needs to be
 * arg2: psc: the prescalar value used by the current system
 */
uint32_t microsecs_to_counter(uint32_t time_length);

/*percentage_arr to perpare an array for arr values
 */
uint32_t get_arr(void);

uint32_t calc_reset_pause(void);

