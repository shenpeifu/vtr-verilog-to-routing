#ifndef __POWER_UTIL_H__
#define __POWER_UTIL_H__

/************************* INCLUDES *********************************/
#include "power.h"
#include "power_components.h"

/************************* FUNCTION DECLARATIONS ********************/
void power_zero_usage(t_power_usage * power_usage);

void power_add_usage(t_power_usage * dest, t_power_usage * src);

float power_usage_sum(t_power_usage * power_usage);

void power_log_msg(enum e_power_log_type log_type, char * msg);

char * transistor_type_name(e_tx_type type);

float pin_density(t_pb * pb, t_pb_graph_pin * pin);

float pin_prob(t_pb * pb, t_pb_graph_pin * pin);

boolean mux_find_selector_values(int * selector_values, t_mux_node * mux_node,
		int selected_input_pin);

int calc_buffer_num_stages(float final_stage_size, float desired_stage_effort);

float calc_buffer_stage_effort(int N, float final_stage_size);

#endif
