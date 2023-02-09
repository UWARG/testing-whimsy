#include "ppm.h"

void __init__(void) {
	counter = BASE_FREQUENCY / (PSC_VALUE + CORRECTION);//calculate the frequency used
	counter_to_microsec = counter / SEC_TO_MICROSEC;//calculate the number of micro sec per tick
	ccr_value = (uint32_t)(counter_to_microsec * PULSE_WIDTH);//ccr_Value is a constant
	ppm_arr_gen();
}

uint32_t microsecs_to_counter(uint32_t time_length) {
	return (uint32_t)(time_length * counter_to_microsec);
}


void ppm_arr_gen(void) {
	for(int i = 0; i < channel_used; i++) {
		time_output[i] = microsecs_to_counter((uint32_t)(MIN_PULSE_WIDTH+user_input[i]*DOWN_INTERVAL));
	}
	time_output[channel_used] = calc_reset_pause();
}

uint32_t calc_reset_pause(void) {
	float remaining_out = (float)channel_used;
	float summation;
	for(int i = 0; i < channel_used; i++) {
		summation = summation + user_input[i];
	}

	remaining_out = remaining_out - summation;

	return microsecs_to_counter((uint32_t)(remaining_out*DOWN_INTERVAL+MIN_RESET_PULSE));
}

