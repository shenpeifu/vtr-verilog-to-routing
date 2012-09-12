/************************* INCLUDES *********************************/
#include <string.h>

#include "power_util.h"
#include "globals.h"

/************************* GLOBALS **********************************/

/************************* FUNCTION DECLARATIONS*********************/
static void log_msg(t_log * log_ptr, char * msg);

/************************* FUNCTION DEFINITIONS *********************/
void power_zero_usage(t_power_usage * power_usage) {
	power_usage->dynamic = 0.;
	power_usage->leakage = 0.;
}

void power_add_usage(t_power_usage * dest, t_power_usage * src) {
	dest->dynamic += src->dynamic;
	dest->leakage += src->leakage;
}

float power_usage_sum(t_power_usage * power_usage) {
	return power_usage->dynamic + power_usage->leakage;
}

void power_log_msg(enum e_power_log_type log_type, char * msg) {
	log_msg(&g_power_output->logs[log_type], msg);
}

char * transistor_type_name(e_tx_type type) {
	if (type == NMOS) {
		return "NMOS";
	} else if (type == PMOS) {
		return "PMOS";
	} else {
		return "Unknown";
	}
}

float pin_density(t_pb * pb, t_pb_graph_pin * pin) {
	float density = 0.;

	if (pb) {
		int net_num;
		net_num = pb->rr_graph[pin->pin_count_in_cluster].net_num;

		if (net_num != OPEN) {
			density = vpack_net[net_num].density;
		}
	}

	return density;
}

float pin_prob(t_pb * pb, t_pb_graph_pin * pin) {
	/* Assumed pull-up on unused interconnect */
	float prob = 1.;

	if (pb) {
		int net_num;
		net_num = pb->rr_graph[pin->pin_count_in_cluster].net_num;

		if (net_num != OPEN) {
			prob = vpack_net[net_num].probability;
		}
	}

	return prob;
}

boolean mux_find_selector_values(int * selector_values, t_mux_node * mux_node,
		int selected_input_pin) {
	if (mux_node->level == 0) {
		if ((selected_input_pin >= mux_node->starting_pin_idx)
				&& (selected_input_pin
						<= (mux_node->starting_pin_idx + mux_node->num_inputs))) {
			selector_values[mux_node->level] = selected_input_pin
					- mux_node->starting_pin_idx;
			return TRUE;
		}
	} else {
		int input_idx;
		for (input_idx = 0; input_idx < mux_node->num_inputs; input_idx++) {
			if (mux_find_selector_values(selector_values,
					&mux_node->children[input_idx], selected_input_pin)) {
				selector_values[mux_node->level] = input_idx;
				return TRUE;
			}
		}
	}
	return FALSE;
}

static void log_msg(t_log * log_ptr, char * msg) {
	int msg_idx;

	/* Check if this message is already in the log */
	for (msg_idx = 0; msg_idx < log_ptr->num_messages; msg_idx++) {
		if (strcmp(log_ptr->messages[msg_idx], msg) == 0) {
			return;
		}
	}

	if (log_ptr->num_messages <= MAX_LOGS) {
		log_ptr->num_messages++;
		log_ptr->messages = my_realloc(log_ptr->messages,
				log_ptr->num_messages * sizeof(char*));
	} else {
		/* Can't add any more messages */
		return;
	}

	if (log_ptr->num_messages == (MAX_LOGS + 1)) {
		const char * full_msg = "\n***LOG IS FULL***\n";
		log_ptr->messages[log_ptr->num_messages - 1] = my_calloc(
				strlen(full_msg) + 1, sizeof(char));
		strncpy(log_ptr->messages[log_ptr->num_messages - 1], full_msg,
				strlen(full_msg));
	} else {
		log_ptr->messages[log_ptr->num_messages - 1] = my_calloc(
				strlen(msg) + 1, sizeof(char));
		strncpy(log_ptr->messages[log_ptr->num_messages - 1], msg, strlen(msg));
	}
}

/*
 * Func Name	: 	calc_buffer_num_stages
 *
 * Description	:	Calculates the number of buffer stages required, to achieve a given buffer fanout
 *
 * Arguments	:	final_stage_size - size of the final stage of the buffer (relative to min-size buffer)
 * 					desired_stage_effort - target fanout gain of each stage of the buffer
 *
 * Returns		:	Number of stages
 */
int calc_buffer_num_stages(float final_stage_size, float desired_stage_effort) {
	int N = 1;

	if (final_stage_size <= 1.0) {
		N = 1;
	} else if (final_stage_size < desired_stage_effort)
		N = 2;
	else {
		N = log(final_stage_size) / log(desired_stage_effort) + 1;

		/* Based on the above, we could use N or (N+1) buffer stages
		 * N stages will require gain > 4
		 * N+1 stages will require gain < 4
		 * Check which provides a effort closer to 4. */
		if (fabs(calc_buffer_stage_effort(N + 1, final_stage_size) - 4)
				< fabs(calc_buffer_stage_effort(N, final_stage_size) - 4)) {
			N++;
		}
	}

	return N;
}

/*
 * Func Name	: 	calc_buffer_stage_effort
 *
 * Description	: 	Calculates the required effort of each stage of a buffer to achieve an overall fanout
 *
 * Arguments	: 	N - number of buffer stages
 * 					final_stage_size - size of the final buffer stage
 *
 * Returns		: 	Effort by each stage
 */
float calc_buffer_stage_effort(int N, float final_stage_size) {
	if (N > 1)
		return pow(final_stage_size, (1.0 / ((double) N - 1)));
	else
		return 1.0;
}

