#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <math.h>

#include "power.h"
#include "physical_types.h"
#include "globals.h"
#include "rr_graph.h"
#include "vpr_utils.h"
#include "ezxml.h"
#include "read_xml_util.h"

#define PRINT_SPICE_COMPARISON 0

#define CONVERT_NM_PER_M 1000000000
#define CONVERT_UM_PER_M 1000000

/* Future */
/* TODO_POWER - Analyze run-time */
/* TODO_POWER - What if packing is run separately? */
/* TODO_POWER - Memory usage of new route algorithm */

/* Immediate */
/* TODO_POWER - LUT driver autosize */
/* TODO_POWER - LUT fracture input sharing fix */
/* TODO_POWER - Clock buffer instead of INV */
/* TODO_POWER - Buffer sense stage Y/N parameter */

/************************* Defines **********************************/
#define POWER_LUT_SLOW
#if (!(defined(POWER_LUT_SLOW) || defined(POWER_LUT_FAST)))
#define POWER_LUT_FAST
#endif

static const float CM_PER_M = 100;
static const float E0 = 8.854187e-12;
static const float Esi = 103.41690416e-12; /* 11.68 * E0 */
static const float q = 1.602177e-19;
static const float Kb = 1.3806503e-23;

/************************* STRUCTS/ENUMS ****************************/
typedef struct s_power_breakdown t_power_breakdown;

struct s_power_breakdown {
	t_power_usage * elements;
};

typedef enum {
	POWER_CIRCUIT_TYPE_CMOS = 0, POWER_CIRCUIT_TYPE_PASS
} e_power_circuit_type;

typedef enum {
	POWER_ELEMENT_IGNORE = 0,
	POWER_ELEMENT_TOTAL,

	POWER_ELEMENT_ROUTING,
	POWER_ELEMENT_ROUTE_SB,
	POWER_ELEMENT_ROUTE_BUFFER,
	POWER_ELEMENT_ROUTE_CB,
	POWER_ELEMENT_ROUTE_GLB_WIRE,

	POWER_ELEMENT_CLOCK,
	POWER_ELEMENT_CLOCK_BUFFER,
	POWER_ELEMENT_CLOCK_WIRE,

	POWER_ELEMENT_TILES,
	POWER_ELEMENT_LOCAL_INTERC,
	POWER_ELEMENT_LOCAL_WIRE,
	POWER_ELEMENT_FF,
	POWER_ELEMENT_LUT,
	POWER_ELEMENT_LUT_DRIVER,
	POWER_ELEMENT_LUT_MUX,
	POWER_ELEMENT_LUT_RESTORER,

	POWER_ELEMENT_MAX_NUM
} e_power_element_type;

typedef struct s_log {
	char * name;
	char ** messages;
	int num_messages;
} t_log;

typedef struct s_power_output {
	FILE * out;
	t_log * logs;
	int num_logs;
} t_power_output;

typedef struct s_power_commonly_used {
	float NMOS_1X_C_gate_max;
	float NMOS_1X_C_drain_max;
	float NMOS_1X_C_source_max;
	float NMOS_1X_C_gate_avg;
	float NMOS_1X_C_drain_avg;
	float NMOS_1X_C_source_avg;
	float NMOS_1X_leakage;
	float NMOS_2X_leakage;
	float PMOS_1X_C_gate_max;
	float PMOS_1X_C_drain_max;
	float PMOS_1X_C_source_max;
	float PMOS_1X_C_gate_avg;
	float PMOS_1X_C_drain_avg;
	float PMOS_1X_C_source_avg;
	float PMOS_1X_leakage;
	float PMOS_2X_leakage;
	float INV_1X_C_in;
	float INV_1X_C;
	float INV_2X_C;

	int max_mux_size;
	t_mux_arch * mux_arch;

	int max_IPIN_fanin;

	int max_seg_fanout;
	int max_seg_to_IPIN_fanout;
	int max_seg_to_seg_fanout;

	float tile_length;

	float SRAM_bit_size;

	int num_sb_buffers;
	float total_sb_buffer_size;

	int num_cb_buffers;
	float total_cb_buffer_size;
} t_power_commonly_used;

enum e_power_log_type {
	POWER_LOG_ERROR, POWER_LOG_WARNING, POWER_LOG_NUM_TYPES
};

/************************* GLOBALS **********************************/

static t_power_arch * g_power_arch;
static t_power_opts * g_power_opts;
static t_power_output * g_power_output;
static t_power_commonly_used * g_power_common;
static t_power_breakdown * g_power_breakdown;
static t_transistor_inf * g_transistor_last_searched;

/************************* Function Declarations ********************/
static float power_count_transistors_pb_type(t_pb_type * pb_type);

void power_add_usage(t_power_usage * dest, t_power_usage * src);

void process_transistor_info(ezxml_t parent);

char * interconnect_type_name(enum e_interconnect type);

void power_print_element_usage(char * name, e_power_element_type type,
		e_power_element_type parent_type, int intdent_level);

static float power_count_transistors_LUT(int LUT_size);

int compare_transistor_size(const void * key_void, const void * elem_void);

void output_log(t_log * log_ptr, FILE * fp);

char * alloc_SRAM_values_from_truth_table(int LUT_size,
		t_linked_vptr * truth_table);

void int_2_binary_str(char * binary_str, int value, int str_length);
float clb_net_density(int net_idx);
float clb_net_prob(int net_idx);

static float power_SRAM_bit_size(void);

void power_calc_pb(t_power_usage * power_usage, t_pb * pb,
		t_pb_graph_node * pb_graph_node);

static float power_count_transistors_tiles(void);

static void power_calc_routing(t_power_usage * power_usage);

void dealloc_mux_graph_recursive(t_mux_node * node);

static void power_calc_MUX2_transmission(t_power_usage * power_usage,
		float * in_dens, float * in_prob, float sel_dens, float sel_prob,
		float out_dens);

void power_print_element_usage_close(char * name, int indent_level);

void power_print_spice_comparison(void);

static float power_transistor_area(float num_transistors);

void power_reset_tile_usage(void);

void power_add_usage_element(t_power_usage * dest, t_power_usage * src,
		e_power_element_type element_idx);

static void power_zero_usage(t_power_usage * power_usage);

static void power_calc_level_restorer(t_power_usage * power_usage,
		float in_density, float in_probability);

void log_msg(t_log * log_ptr, char * msg);

static void power_calc_interconnect(t_power_usage * power_usage, t_pb * pb,
		t_interconnect_pins * interc_pins, float interc_length);

void alloc_and_load_mux_graph_recursive(t_mux_node * node,
		int num_primary_inputs, int level, int starting_pin_idx);

float pin_density(t_pb * pb, t_pb_graph_pin * pin);

float pin_prob(t_pb * pb, t_pb_graph_pin * pin);

static float power_usage_combined(t_power_usage * power_usage);

void dealloc_mux_graph(t_mux_node * node);

void power_log_msg(enum e_power_log_type log_type, char * msg);

void output_logs(FILE * fp, t_log * logs, int num_logs);

char binary_not(char c);

void init_mux_arch_default(t_mux_arch * mux_arch, int levels, int num_inputs);

void power_calc_LUT(t_power_usage * power_usage, int LUT_size,
		char * SRAM_values, float * input_densities,
		float * input_probabilities);

t_mux_node * alloc_and_load_mux_graph(int num_inputs, int levels);

void power_calc_buffer(t_power_usage * power_usage, float size, float in_dens,
		float in_probability);

int calc_buffer_num_stages(float final_stage_size, float desired_stage_effort);

float calc_buffer_stage_effort(int N, float final_stage_size);

char * transistor_type_name(e_tx_type type);

float get_NFS(e_tx_type transistor_type, int temperature);

static void power_calc_clock(t_power_usage * power_usage, t_clocks * clocks);

static float power_count_transistors_buffer(float buffer_size);

static void power_calc_clock_single(t_power_usage * power_usage,
		t_clock_inf * clock_inf);

void power_read_cmos_tech_behavior(void);

boolean file_exists(const char * filename);

void power_calc_tile_usage(t_power_usage * cb_power_usage);

void power_calc_pb_recursive(t_power_usage * power_usage, t_pb * pb,
		t_pb_graph_node * pb_graph_node, boolean calc_dynamic,
		boolean calc_leakage);

void power_calc_blif_primitive(t_power_usage * power_usage, t_pb * pb,
		t_pb_graph_node * pb_graph_node, boolean calc_dynamic,
		boolean calc_leakage);

void power_calc_MUX(t_power_usage * power_usage, t_mux_arch * mux_arch,
		float * in_density, float * in_prob, int selected_input);

float power_calc_transistor_leakage(e_tx_type transistor_type, float size);

void power_calc_transistor_capacitance(float *C_drain, float *C_source,
		float *C_gate, e_tx_type transistor_type, float size,
		e_power_circuit_type circuit_type);

void power_calc_INV(t_power_usage * power_usage, float in_density,
		float in_probability, float PMOS_size, float NMOS_size);

void power_reset_pb_type(t_pb_type * pb_type);

void power_calc_INV1(t_power_usage * power_usage, float in_density,
		float in_probability);

int binary_str_to_integer(int binary_length, char *binary_str);

void power_calc_INV2(t_power_usage * power_usage, float in_density,
		float in_probability);

void power_calc_MUX_node(t_power_usage * power_usage, float * out_dens,
		float * out_prob, t_mux_node * mux_node, t_mux_arch * mux_arch,
		int * selector_values, float * primary_input_density,
		float * primary_input_prob);

static void power_calc_FF(t_power_usage * power_usage, float D_dens,
		float D_prob, float Q_dens, float Q_prob, float clk_dens,
		float clk_prob);

void power_print_pb_usage_recursive(FILE * fp, t_pb_type * type,
		int indent_level, float parent_power, float total_power);

static void power_calc_MUX2(t_power_usage * power_usage, float Vin, float Vout,
		float out_density, float * in_density, float * in_prob, float sel_dens,
		float sel_prob);

boolean power_find_transistor_info(t_transistor_size_inf ** lower,
		t_transistor_size_inf ** upper, e_tx_type type, float size);

boolean mux_find_selector_values(int * selector_values, t_mux_node * mux_node,
		int selected_input_pin);

void power_print_pb_usage(FILE * fp);

void integer_to_SRAMvalues(int SRAM_num, int input_integer, char SRAM_values[]);

/************************* Function Definitions *********************/

int compare_transistor_size(const void * key_void, const void * elem_void) {
	const t_transistor_size_inf * key = key_void;
	const t_transistor_size_inf * elem = elem_void;
	const t_transistor_size_inf * next;

	/* Check if we are comparing against the last element */
	if (elem
			== &g_transistor_last_searched->size_inf[g_transistor_last_searched->num_size_entries
					- 1]) {
		/* Match if the desired value is larger than the largest item in the list */
		if (key->size >= elem->size) {
			return 0;
		} else {
			return -1;
		}
	}

	/* Check if we are comparing against the first element */
	if (elem == &g_transistor_last_searched->size_inf[0]) {
		/* Match the smallest if it is smaller than the smallest */
		if (key->size < elem->size) {
			return 0;
		}
	}

	/* Check if the key is between elem and the next element */
	next = elem + 1;
	if (key->size > next->size) {
		return 1;
	} else if (key->size < elem->size) {
		return -1;
	} else {
		return 0;
	}

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

boolean power_find_transistor_info(t_transistor_size_inf ** lower,
		t_transistor_size_inf ** upper, e_tx_type type, float size) {
	char msg[1024];
	t_transistor_size_inf key;
	t_transistor_size_inf * found;
	t_transistor_inf * trans_info;
	float min_size, max_size;
	boolean error = FALSE;

	key.size = size;

	if (type == NMOS) {
		trans_info = &g_power_arch->NMOS_tx_record;
	} else if (type == PMOS) {
		trans_info = &g_power_arch->PMOS_tx_record;
	} else {
		assert(0);
	}

	/* No transistor data exists */
	if (trans_info->size_inf == NULL) {
		power_log_msg(POWER_LOG_ERROR,
				"No transistor information exists.  Cannot determine transistor properties.");
		error = TRUE;
		return error;
	}

	g_transistor_last_searched = trans_info;
	min_size = trans_info->size_inf[0].size;
	max_size = trans_info->size_inf[trans_info->num_size_entries - 1].size;

	found = bsearch(&key, trans_info->size_inf, trans_info->num_size_entries,
			sizeof(t_transistor_size_inf), compare_transistor_size);
	assert(found);

	/* Too small */
	if (size < min_size) {
		assert(found == &trans_info->size_inf[0]);
		sprintf(msg,
				"Using %s transistor of size '%f', which is smaller than the smallest modeled transistor (%f) in the technology behavior file.",
				transistor_type_name(type), size, min_size);
		power_log_msg(POWER_LOG_WARNING, msg);
		*lower = NULL;
		*upper = found;
	} else if (size > max_size) {
		assert(
				found == &trans_info->size_inf[trans_info->num_size_entries - 1]);
		sprintf(msg,
				"Using %s transistor of size '%f', which is larger than the largest modeled transistor (%f) in the technology behavior file.",
				transistor_type_name(type), size, max_size);
		power_log_msg(POWER_LOG_WARNING, msg);
		*lower = found;
		*upper = NULL;
	} else {
		*lower = found;
		*upper = found + 1;
	}

	return error;
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

void power_calc_MUX(t_power_usage * power_usage, t_mux_arch * mux_arch,
		float * in_density, float * in_prob, int selected_input) {
	float output_density;
	float output_prob;
	boolean found;
	int * selector_values = my_calloc(mux_arch->levels, sizeof(int));

	assert(selected_input != OPEN);

	power_usage->dynamic = 0.;
	power_usage->leakage = 0.;

	/* Find selection index at each level */

	found = mux_find_selector_values(selector_values, mux_arch->mux_graph_head,
			selected_input);
	assert(found);

	/* Calculate power of the multiplexor stages, from final stage, to first stages */
	power_calc_MUX_node(power_usage, &output_density, &output_prob,
			mux_arch->mux_graph_head, mux_arch, selector_values, in_density,
			in_prob);

	free(selector_values);
}

void integer_to_SRAMvalues(int SRAM_num, int input_integer, char SRAM_values[]) {
	char binary_str[20];
	int binary_str_counter;
	int local_integer;
	int i;

	binary_str_counter = 0;

	local_integer = input_integer;

	while (local_integer > 0) {
		if (local_integer % 2 == 0) {
			SRAM_values[binary_str_counter++] = '0';
		} else {
			SRAM_values[binary_str_counter++] = '1';
		}
		local_integer = local_integer / 2;
	}

	while (binary_str_counter < SRAM_num) {
		SRAM_values[binary_str_counter++] = '0';
	}

	SRAM_values[binary_str_counter] = '\0';

	for (i = 0; i < binary_str_counter; i++) {
		binary_str[i] = SRAM_values[binary_str_counter - 1 - i];
	}

	binary_str[binary_str_counter] = '\0';

}

int binary_str_to_integer(int binary_length, char *binary_str) {
	int i;
	int integer_value;
	int temp_value;

	integer_value = 0;

	for (i = 0; i < binary_length; i++) {
		if (binary_str[i] == '1') {
			temp_value = (1 << (binary_length - i - 1));
			integer_value += temp_value;
		}
	}

	return (integer_value);
}

#if 0
void calc_MUX_prob_and_density (
		float *out_prob,
		float *out_dens,
		float in_0_prob,
		float in_1_prob,
		float sel_prob,
		float in_0_density,
		float in_1_density,
		float sel_dens,
)
{
	/* Calculations based on the transition density signal model */

	(*out_prob) = in_0_prob * (1 - sel_prob) + in_1_prob * sel_prob;

	(*out_dens) = (1 - sel_prob) * in_0_density + sel_prob * in_1_density +
	((1 - in_0_prob) * in_1_prob + in_0_prob * (1 - in_1_prob)) * sel_dens;
}
#endif

void power_add_usage(t_power_usage * dest, t_power_usage * src) {
	dest->dynamic += src->dynamic;
	dest->leakage += src->leakage;
}

void power_add_usage_element(t_power_usage * dest, t_power_usage * src,
		e_power_element_type element_idx) {
	if (dest) {
		power_add_usage(dest, src);
	}
	power_add_usage(&g_power_breakdown->elements[element_idx], src);
}

void power_calc_LUT(t_power_usage * power_usage, int LUT_size,
		char * SRAM_values, float * input_densities,
		float * input_probabilities) {
	float **internal_prob;
	float **internal_density;
	float power_per_lut;
	float input_related_power;
	int i;
	int level_idx;

	int MUXCounter;
	int SRAMCounter;
	int num_SRAM_bits;

	boolean level_restorer_this_level;
	boolean level_restorer_last_level;

	power_zero_usage(power_usage);

	num_SRAM_bits = 1 << LUT_size;

	/* Initialize internal node data */
	internal_prob = my_calloc(LUT_size + 1, sizeof(float*));
	for (i = 0; i <= LUT_size; i++) {
		internal_prob[i] = my_calloc(1 << (LUT_size - i), sizeof(float));
	}

	internal_density = my_calloc(LUT_size + 1, sizeof(float*));
	for (i = 0; i <= LUT_size; i++) {
		internal_density[i] = my_calloc(1 << (LUT_size - i), sizeof(float));
	}

	/* initialize the SRAM_values in the LUTs */
	/* Notice: the input signals that control the output of
	 the LUT go from LSB to MSB.
	 Please refer to figure B.9 in Vaughn's thesis.
	 In0 is the LSB and IN3 is the MSB. Therefore, the
	 function value needs to be converted to match the
	 LUT SRAM values.
	 */

	/* Initialize internal probabilities/densities from SRAM bits */
	for (i = 0; i < num_SRAM_bits; i++) {
		if (SRAM_values[i] == '0') {
			internal_prob[0][i] = 0.;
		} else {
			internal_prob[0][i] = 1.;
		}
		internal_density[0][i] = 0.;
	}

	power_per_lut = 0.0;
	input_related_power = 0.0;

	for (level_idx = 0; level_idx < LUT_size; level_idx++) {
		t_power_usage driver_power_usage;
		int MUXs_this_level;
		int MUX_idx;
		int reverse_idx = LUT_size - level_idx - 1;
		float Vin;
		float Vout;

		MUXs_this_level = 1 << (reverse_idx);

		power_calc_INV1(&driver_power_usage, input_densities[reverse_idx],
				input_probabilities[reverse_idx]);
		power_add_usage_element(power_usage, &driver_power_usage,
				POWER_ELEMENT_LUT_DRIVER);

		power_calc_INV2(&driver_power_usage, input_densities[reverse_idx],
				input_probabilities[reverse_idx]);
		power_add_usage_element(power_usage, &driver_power_usage,
				POWER_ELEMENT_LUT_DRIVER);

		power_calc_INV2(&driver_power_usage, input_densities[reverse_idx],
				1 - input_probabilities[reverse_idx]);
		power_add_usage_element(power_usage, &driver_power_usage,
				POWER_ELEMENT_LUT_DRIVER);

		MUXCounter = 0;
		SRAMCounter = 0;

		if (level_idx == 0) {
			level_restorer_last_level = TRUE;
		} else {
			level_restorer_last_level = level_restorer_this_level;
		}

		/* Add level restorer after every 2 stages (level_idx %2 == 1)
		 * But if there is an odd # of stages, just put one at the last
		 * stage (level_idx == LUT_size - 1) and not at the stage just before
		 * the last stage (level_idx != LUT_size - 2)
		 */
		if (((level_idx % 2 == 1) && (level_idx != LUT_size - 2))
				|| (level_idx == LUT_size - 1)) {
			level_restorer_this_level = TRUE;
		} else {
			level_restorer_this_level = FALSE;
		}

		/* Calculate Vin,Vout of the muxes at this level */
		Vin = g_power_arch->Vdd;
		Vout = g_power_arch->Vdd;
		if (!level_restorer_this_level) {
			Vout -= g_power_arch->NMOS_tx_record.Vth;
		}
		if (!level_restorer_last_level) {
			Vin -= g_power_arch->NMOS_tx_record.Vth;
		}

		for (MUX_idx = 0; MUX_idx < MUXs_this_level; MUX_idx++) {
			t_power_usage sub_power;
			float out_prob;
			float out_dens;
			float sum_prob = 0;
			int sram_offset = MUX_idx * pow(2, level_idx + 1);
			int sram_per_branch = pow(2, level_idx);
			int branch_lvl_idx;
			int sram_idx;

			out_prob = internal_prob[level_idx][MUX_idx * 2]
					* (1 - input_probabilities[reverse_idx])
					+ internal_prob[level_idx][MUX_idx * 2 + 1]
							* input_probabilities[reverse_idx];

			out_dens = internal_density[level_idx][MUX_idx * 2]
					* (1 - input_probabilities[reverse_idx])
					+ internal_density[level_idx][MUX_idx * 2 + 1]
							* input_probabilities[reverse_idx];

#ifdef POWER_LUT_FAST
			out_dens += ((1 - internal_prob[level_idx][MUX_idx * 2]) * internal_prob[level_idx][MUX_idx * 2 + 1]
					+ internal_prob[level_idx][MUX_idx * 2] * (1 - internal_prob[level_idx][MUX_idx * 2 + 1]))
			* input_densities[reverse_idx];
#elif defined(POWER_LUT_SLOW)
			for (sram_idx = sram_offset;
					sram_idx < sram_offset + sram_per_branch; sram_idx++) {
				float branch_prob = 1.;
				if (SRAM_values[sram_idx]
						== SRAM_values[sram_idx + sram_per_branch]) {
					continue;
				}
				for (branch_lvl_idx = 0; branch_lvl_idx < level_idx;
						branch_lvl_idx++) {
					int branch_lvl_reverse_idx = LUT_size - branch_lvl_idx - 1;
					int even_odd = sram_idx / pow(2, branch_lvl_idx);
					if (even_odd % 2 == 0) {
						branch_prob *= (1
								- input_probabilities[branch_lvl_reverse_idx]);
					} else {
						branch_prob *=
								input_probabilities[branch_lvl_reverse_idx];
					}
				}
				sum_prob += branch_prob;
			}
			out_dens += sum_prob * input_densities[reverse_idx];
#endif
			internal_density[level_idx + 1][MUX_idx] = out_dens;
			internal_prob[level_idx + 1][MUX_idx] = out_prob;

			power_calc_MUX2(&sub_power, Vin, Vout,
					internal_density[level_idx + 1][MUX_idx],
					&internal_density[level_idx][MUX_idx * 2],
					&internal_prob[level_idx][MUX_idx * 2],
					input_densities[reverse_idx],
					input_probabilities[reverse_idx]);
			power_add_usage_element(power_usage, &sub_power,
					POWER_ELEMENT_LUT_MUX);

			if (level_restorer_this_level) {
				/* Level restorer */
				power_calc_level_restorer(&sub_power,
						internal_density[level_idx + 1][MUX_idx],
						internal_prob[level_idx + 1][MUX_idx]);
				power_add_usage_element(power_usage, &sub_power,
						POWER_ELEMENT_LUT_RESTORER);
			}
		}

	}

	for (i = 0; i <= LUT_size; i++) {
		free(internal_prob[i]);
		free(internal_density[i]);
	}
	free(internal_prob);
	free(internal_density);

	power_add_usage_element(NULL, power_usage, POWER_ELEMENT_LUT);

	return;
}

void int_2_binary_str(char * binary_str, int value, int str_length) {
	int i;
	int odd;

	binary_str[str_length] = '\0';

	for (i = str_length - 1; i >= 0; i--, value >>= 1) {
		odd = value % 2;
		if (odd == 0) {
			binary_str[i] = '0';
		} else {
			binary_str[i] = '1';
		}
	}
}

char * alloc_SRAM_values_from_truth_table(int LUT_size,
		t_linked_vptr * truth_table) {
	char * SRAM_values;
	int i;
	int num_SRAM_bits;
	char * binary_str;
	char ** terms;
	char * buffer;
	char * str_loc;
	boolean on_set;
	t_linked_vptr * list_ptr;
	int num_terms;
	int term_idx;
	int bit_idx;
	int dont_care_start_pos;
	boolean used = TRUE;

	num_SRAM_bits = 1 << LUT_size;
	SRAM_values = my_calloc(num_SRAM_bits + 1, sizeof(char));
	SRAM_values[num_SRAM_bits] = '\0';

	if (!truth_table) {
		used = FALSE;
		for (i = 0; i < num_SRAM_bits; i++) {
			SRAM_values[i] = '1';
		}
		return SRAM_values;
	}

	binary_str = my_calloc(LUT_size + 1, sizeof(char));
	buffer = my_calloc(LUT_size + 10, sizeof(char));

	strcpy(buffer, truth_table->data_vptr);

	/* Check if this is an unconnected node - hopefully these will be
	 * ignored by VPR in the future
	 */
	if (strcmp(buffer, " 0") == 0) {
		free(binary_str);
		free(buffer);
		return SRAM_values;
	} else if (strcmp(buffer, " 1") == 0) {
		for (i = 0; i < num_SRAM_bits; i++) {
			SRAM_values[i] = '1';
		}
		free(binary_str);
		free(buffer);
		return SRAM_values;
	}

	/* If the LUT is larger than the terms, the lower significant bits will be don't cares */
	str_loc = strtok(buffer, " \t");
	dont_care_start_pos = strlen(str_loc);

	/* Find out if the truth table provides the ON-set or OFF-set */
	str_loc = strtok(NULL, " \t");
	if (str_loc[0] == '1') {
		on_set = TRUE;
	} else if (str_loc[0] == '0') {
		on_set = FALSE;
	} else {
		assert(0);
	}

	/* Count truth table terms */
	num_terms = 0;
	for (list_ptr = truth_table; list_ptr != NULL; list_ptr = list_ptr->next) {
		num_terms++;
	}
	terms = my_calloc(num_terms, sizeof(char *));

	/* Extract truth table terms */
	for (list_ptr = truth_table, term_idx = 0; list_ptr != NULL; list_ptr =
			list_ptr->next, term_idx++) {
		terms[term_idx] = my_calloc(LUT_size + 1, sizeof(char));

		strcpy(buffer, list_ptr->data_vptr);
		str_loc = strtok(buffer, " \t");
		strcpy(terms[term_idx], str_loc);

		/* Fill don't cares for lower bits (when LUT is larger than term size) */
		for (bit_idx = dont_care_start_pos; bit_idx < LUT_size; bit_idx++) {
			terms[term_idx][bit_idx] = '-';
		}

		/* Verify on/off consistency */
		str_loc = strtok(NULL, " \t");
		if (on_set) {
			assert(str_loc[0] == '1');
		} else {
			assert(str_loc[0] == '0');
		}
	}

	/* Loop through all SRAM bits */
	for (i = 0; i < num_SRAM_bits; i++) {
		/* Set default value */
		if (on_set) {
			SRAM_values[i] = '0';
		} else {
			SRAM_values[i] = '1';
		}

		/* Get binary number representing this SRAM index */
		int_2_binary_str(binary_str, i, LUT_size);

		/* Loop through truth table terms */
		for (term_idx = 0; term_idx < num_terms; term_idx++) {
			boolean match = TRUE;

			for (bit_idx = 0; bit_idx < LUT_size; bit_idx++) {
				if ((terms[term_idx][bit_idx] != '-')
						&& (terms[term_idx][bit_idx] != binary_str[bit_idx])) {
					match = FALSE;
					break;
				}
			}

			if (match) {
				if (on_set) {
					SRAM_values[i] = '1';
				} else {
					SRAM_values[i] = '0';
				}

				/* No need to check the other terms, already matched */
				break;
			}
		}

	}
	free(binary_str);
	free(buffer);
	for (term_idx = 0; term_idx < num_terms; term_idx++) {
		free(terms[term_idx]);
	}
	free(terms);

	return SRAM_values;
}

void power_calc_blif_primitive(t_power_usage * power_usage, t_pb * pb,
		t_pb_graph_node * pb_graph_node, boolean calc_dynamic,
		boolean calc_leakage) {
	t_power_usage sub_power_usage;
	power_usage->dynamic = 0.;
	power_usage->leakage = 0.;

	if (strcmp(pb_graph_node->pb_type->blif_model, ".names") == 0) {
		char * SRAM_values;
		float * input_probabilities;
		float * input_densities;
		int LUT_size;
		int pin_idx;

		assert(pb_graph_node->num_input_ports == 1);

		LUT_size = pb_graph_node->num_input_pins[0];

		input_probabilities = my_calloc(LUT_size, sizeof(float));
		input_densities = my_calloc(LUT_size, sizeof(float));

		for (pin_idx = 0; pin_idx < LUT_size; pin_idx++) {
			t_pb_graph_pin * pin = &pb_graph_node->input_pins[0][pin_idx];

			input_probabilities[pin_idx] = pin_prob(pb, pin);
			input_densities[pin_idx] = pin_density(pb, pin);
		}

		if (pb) {
			SRAM_values = alloc_SRAM_values_from_truth_table(LUT_size,
					logical_block[pb->logical_block].truth_table);
		} else {
			SRAM_values = alloc_SRAM_values_from_truth_table(LUT_size, NULL);
		}
		power_calc_LUT(&sub_power_usage, LUT_size, SRAM_values, input_densities,
				input_probabilities);
		power_add_usage(power_usage, &sub_power_usage);
		free(SRAM_values);
		free(input_probabilities);
		free(input_densities);
	} else if (strcmp(pb_graph_node->pb_type->blif_model, ".latch") == 0) {
		t_pb_graph_pin * D_pin = &pb_graph_node->input_pins[0][0];
		t_pb_graph_pin * Q_pin = &pb_graph_node->output_pins[0][0];
		/*t_pb_graph_pin * clk_pin = &pb_graph_node->clock_pins[0][0];*/
		float D_dens = 0.;
		float D_prob = 0.;
		float Q_prob = 0.;
		float Q_dens = 0.;
		float clk_dens = 0.;
		float clk_prob = 0.;

		D_dens = pin_density(pb, D_pin);
		D_prob = pin_prob(pb, D_pin);
		Q_dens = pin_density(pb, Q_pin);
		Q_prob = pin_prob(pb, Q_pin);

		assert(g_arch->clocks->num_global_clock == 1);
		clk_dens = g_arch->clocks->clock_inf[0].density;
		clk_prob = g_arch->clocks->clock_inf[0].prob;
		/*clk_dens = pin_density(pb, clk_pin);
		 clk_prob = pin_prob(pb, clk_pin);*/

		power_calc_FF(&sub_power_usage, D_dens, D_prob, Q_dens, Q_prob,
				clk_dens, clk_prob);
		power_add_usage_element(power_usage, &sub_power_usage,
				POWER_ELEMENT_FF);
	} else {
		char msg[BUFSIZE];
		power_usage->dynamic = 0.;
		power_usage->leakage = 0.;

		if (calc_dynamic) {
			sprintf(msg, "No dynamic power defined for BLIF model: %s",
					pb_graph_node->pb_type->blif_model);
			power_log_msg(POWER_LOG_WARNING, msg);
		}
		if (calc_leakage) {
			sprintf(msg, "No leakage power defined for BLIF model: %s",
					pb_graph_node->pb_type->blif_model);
			power_log_msg(POWER_LOG_WARNING, msg);
		}

	}

	if (!calc_dynamic) {
		power_usage->dynamic = 0.;
	}
	if (!calc_leakage) {
		power_usage->leakage = 0.;
	}
}

void power_calc_pb(t_power_usage * power_usage, t_pb * pb,
		t_pb_graph_node * pb_graph_node) {
	power_calc_pb_recursive(power_usage, pb, pb_graph_node, TRUE, TRUE);
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

static void power_calc_interconnect(t_power_usage * power_usage, t_pb * pb,
		t_interconnect_pins * interc_pins, float interc_length) {
	int pin_idx;
	int out_port_idx;
	int in_port_idx;
	float * input_densities;
	float * input_prob;
	float C_wire;
	float dens;
	t_interconnect * interc = interc_pins->interconnect;
	t_power_usage sub_power_usage;

	power_zero_usage(power_usage);

	/* Ensure port/pins are structured as expected */
	switch (interc_pins->interconnect->type) {
	case DIRECT_INTERC:
		assert(interc_pins->interconnect->num_input_ports == 1);
		assert(interc_pins->interconnect->num_output_ports == 1);
		break;
	case MUX_INTERC:
		assert(interc_pins->interconnect->num_output_ports == 1);
		break;
	case COMPLETE_INTERC:
		break;
	}

	/* Interconnect Wire Capacitances */

	/* Assume input/output wire length are each half of interc_length */
	C_wire = 0.5 * interc_length * g_power_arch->C_wire_local;

	power_zero_usage(&sub_power_usage);

	for (out_port_idx = 0;
			out_port_idx < interc_pins->interconnect->num_output_ports;
			out_port_idx++) {
		for (pin_idx = 0;
				pin_idx < interc_pins->interconnect->num_pins_per_port;
				pin_idx++) {

			/* Wires to inputs */
			for (in_port_idx = 0;
					in_port_idx < interc_pins->interconnect->num_input_ports;
					in_port_idx++) {
				dens = pin_density(pb,
						interc_pins->input_pins[in_port_idx][pin_idx]);
				sub_power_usage.dynamic = power_calc_dynamic(C_wire, dens);
				power_add_usage_element(power_usage, &sub_power_usage,
						POWER_ELEMENT_LOCAL_WIRE);
			}

			/* Wire from output */
			dens = pin_density(pb,
					interc_pins->output_pins[out_port_idx][pin_idx]);
			sub_power_usage.dynamic = power_calc_dynamic(C_wire, dens);
			power_add_usage_element(power_usage, &sub_power_usage,
					POWER_ELEMENT_LOCAL_WIRE);
		}
	}

	/* Buffers */
#if 0
	for (pin_idx = 0; pin_idx < interc->num_pins_per_port; pin_idx++) {
		for (in_port_idx = 0; in_port_idx < interc->num_input_ports;
				in_port_idx++) {
			power_calc_buffer(&sub_power_usage, interc->buffer_in_size,
					pin_density(pb,
							interc_pins->input_pins[in_port_idx][pin_idx]),
					pin_prob(pb,
							interc_pins->input_pins[in_port_idx][pin_idx]));
			power_add_usage(power_usage, &sub_power_usage);
		}
		for (out_port_idx = 0; out_port_idx < interc->num_output_ports;
				out_port_idx++) {
			power_calc_buffer(&sub_power_usage, interc->buffer_out_size,
					pin_density(pb,
							interc_pins->output_pins[out_port_idx][pin_idx]),
					pin_prob(pb,
							interc_pins->output_pins[out_port_idx][pin_idx]));
			power_add_usage(power_usage, &sub_power_usage);
		}
	}
#endif

	switch (interc_pins->interconnect->type) {
	case DIRECT_INTERC:
		break;
	case MUX_INTERC:
	case COMPLETE_INTERC:
		input_densities = my_calloc(interc->num_input_ports, sizeof(float));
		input_prob = my_calloc(interc->num_input_ports, sizeof(float));

		for (out_port_idx = 0; out_port_idx < interc->num_output_ports;
				out_port_idx++) {
			for (pin_idx = 0; pin_idx < interc->num_pins_per_port; pin_idx++) {
				t_power_usage MUX_power;
				int selected_input = OPEN;

				/* Clear input densities */
				for (in_port_idx = 0; in_port_idx < interc->num_input_ports;
						in_port_idx++) {
					input_densities[in_port_idx] = 0.;
					input_prob[in_port_idx] = 0.;
				}

				if (pb) {
					int output_pin_net =
							pb->rr_graph[interc_pins->output_pins[out_port_idx][pin_idx]->pin_count_in_cluster].net_num;

					if (output_pin_net == OPEN) {
						selected_input = 0;
					} else {
						for (in_port_idx = 0;
								in_port_idx < interc->num_input_ports;
								in_port_idx++) {
							t_pb_graph_pin * input_pin =
									interc_pins->input_pins[in_port_idx][pin_idx];
							int input_pin_net =
									pb->rr_graph[input_pin->pin_count_in_cluster].net_num;

							/* Find input pin that connects through the mux to the output pin */
							if (output_pin_net == input_pin_net) {
								selected_input = in_port_idx;
							}

							/* Initialize input densities */
							if (input_pin_net != OPEN) {
								input_densities[in_port_idx] = pin_density(pb,
										input_pin);
								input_prob[in_port_idx] = pin_prob(pb,
										input_pin);
							}
						}

						/* Check that the input pin was found with a matching net to the output pin */
						assert(selected_input != OPEN);
					}
				} else {
					selected_input = 0;
				}

				power_calc_MUX(&MUX_power, interc_pins->interconnect->mux_arch,
						input_densities, input_prob, selected_input);

				power_add_usage_element(power_usage, &MUX_power,
						POWER_ELEMENT_LOCAL_INTERC);
			}
		}

		free(input_densities);
		free(input_prob);
		break;
	}

	power_add_usage(&interc_pins->interconnect->power_usage, power_usage);
}

void mux_arch_fix_levels(t_mux_arch * mux_arch) {
	while ((pow(2, mux_arch->levels) > mux_arch->num_inputs)
			&& (mux_arch->levels > 1)) {
		mux_arch->levels--;
	}
}

void power_calc_pb_recursive(t_power_usage * power_usage, t_pb * pb,
		t_pb_graph_node * pb_graph_node, boolean calc_dynamic,
		boolean calc_leakage) {
	t_power_usage sub_power_usage;
	int pb_type_idx;
	int pb_idx;
	int interc_idx;
	int pb_mode;
	boolean ignore_interconnect = FALSE;
	boolean continue_dynamic = calc_dynamic;
	boolean continue_leakage = calc_leakage;

	power_zero_usage(power_usage);

	if (pb) {
		pb_mode = pb->mode;
	} else {
		/* Default mode if not initialized (will only affect leakage power) */
		pb_mode = pb_graph_node->pb_type->leakage_default_mode;

		/* No dynamic power if this block is not used */
		continue_dynamic = FALSE;
	}

	/* Check if leakage power is explicitly specified */
	if (continue_leakage
			&& (pb_graph_node->pb_type->power_leakage_type == LEAKAGE_PROVIDED)) {
		power_usage->leakage += pb_graph_node->pb_type->power_leakage;
		continue_leakage = FALSE;
	}

	/* Check if dynamic power is explicitly specified, or internal capacitance is
	 * specified
	 */
	if (continue_dynamic) {
		assert(pb != NULL);

		if (pb_graph_node->pb_type->power_dynamic_type == DYNAMIC_PROVIDED) {
			power_usage->dynamic += pb_graph_node->pb_type->power_dynamic;
			continue_dynamic = FALSE;
		} else if (pb_graph_node->pb_type->power_dynamic_type
				== DYNAMIC_C_INTERNAL) {
			/* Just take the average density of inputs pins and use
			 * that with user-defined block capacitance and leakage */
			float density = 0.;

			int port_idx;
			int pin_idx;
			int num_pins = 0;
			for (port_idx = 0; port_idx < pb_graph_node->num_input_ports;
					port_idx++) {
				for (pin_idx = 0;
						pin_idx < pb_graph_node->num_input_pins[port_idx];
						pin_idx++) {
					if (pb->rr_graph[pb_graph_node->input_pins[port_idx][pin_idx].pin_count_in_cluster].net_num
							!= OPEN) {
						density += pin_density(pb,
								&pb_graph_node->input_pins[port_idx][pin_idx]);
					}
					num_pins++;
				}
			}

			if (num_pins != 0) {
				density = density / num_pins;
			}
			power_usage->dynamic += power_calc_dynamic(
					pb_graph_node->pb_type->C_internal, density);
			continue_dynamic = FALSE;
		}
	}

	/* Check if done dynamic and leakage power calculation */
	if (!(continue_dynamic || continue_leakage)) {
		return;
	}

	if (pb_graph_node->pb_type->class_type == LUT_CLASS) {
		/* LUTs will have a child node that is used for routing purposes
		 * For the routing algorithms it is completely connected; however,
		 * this interconnect does not exist in FPGA hardware and should
		 * be ignored for power calculations. */
		ignore_interconnect = TRUE;
	}

	if (pb_graph_node->pb_type->num_modes == 0) {
		/* This is a leaf node */
		/* All leaf nodes should implement a hardware primitive from the blif file */
		assert(pb_graph_node->pb_type->blif_model);

		power_calc_blif_primitive(&sub_power_usage, pb, pb_graph_node,
				continue_dynamic, continue_leakage);
		power_add_usage(power_usage, &sub_power_usage);
	} else {
		/* This node had children.  The power of this node is the sum of:
		 *  - Interconnect from parent to children
		 *  - Child nodes
		 */
		if (!ignore_interconnect) {
			for (interc_idx = 0;
					interc_idx
							< pb_graph_node->pb_type->modes[pb_mode].num_interconnect;
					interc_idx++) {
				float interc_length;

				/* Determine the length this interconnect covers.
				 * This is assumed to be half of the square length of the
				 * interconnect area */
				interc_length =
						0.5
								* sqrt(
										power_transistor_area(
												pb_graph_node->pb_type->transistor_cnt_interc));

				power_calc_interconnect(&sub_power_usage, pb,
						&pb_graph_node->interconnect_pins[pb_mode][interc_idx],
						interc_length);
				power_add_usage(power_usage, &sub_power_usage);
			}
		}

		for (pb_type_idx = 0;
				pb_type_idx
						< pb_graph_node->pb_type->modes[pb_mode].num_pb_type_children;
				pb_type_idx++) {
			for (pb_idx = 0;
					pb_idx
							< pb_graph_node->pb_type->modes[pb_mode].pb_type_children[pb_type_idx].num_pb;
					pb_idx++) {
				t_pb * child_pb = NULL;
				t_pb_graph_node * child_pb_graph_node;

				if (pb && pb->child_pbs[pb_type_idx][pb_idx].name) {
					/* Child is initialized */
					child_pb = &pb->child_pbs[pb_type_idx][pb_idx];
				}
				child_pb_graph_node =
						&pb_graph_node->child_pb_graph_nodes[pb_mode][pb_type_idx][pb_idx];

				/* Recursive call for children */
				power_calc_pb_recursive(&sub_power_usage, child_pb,
						child_pb_graph_node, continue_dynamic,
						continue_leakage);
				power_add_usage(power_usage, &sub_power_usage);
			}
		}
		power_add_usage(&pb_graph_node->pb_type->modes[pb_mode].power_usage,
				power_usage);
	}

	power_add_usage(&pb_graph_node->pb_type->power_usage, power_usage);
}

#if 0
void calc_leakage_complexblocks_unused(float * block_leakage) {
	int type_idx;
	for (type_idx = 0; type_idx < num_types; type_idx++) {
		t_power_usage pb_power;

		if (&type_descriptors[type_idx] == EMPTY_TYPE) {
			continue;
		}

		power_calc_pb(&pb_power, NULL,
				type_descriptors[type_idx].pb_graph_head);

		block_leakage[type_idx] = pb_power.leakage;
	}
}
#endif

static void power_zero_usage(t_power_usage * power_usage) {
	power_usage->dynamic = 0.;
	power_usage->leakage = 0.;
}

void power_reset_pb_type(t_pb_type * pb_type) {
	int mode_idx;
	int child_idx;
	int interc_idx;

	power_zero_usage(&pb_type->power_usage);

	for (mode_idx = 0; mode_idx < pb_type->num_modes; mode_idx++) {
		power_zero_usage(&pb_type->modes[mode_idx].power_usage);

		for (child_idx = 0;
				child_idx < pb_type->modes[mode_idx].num_pb_type_children;
				child_idx++) {
			power_reset_pb_type(
					&pb_type->modes[mode_idx].pb_type_children[child_idx]);
		}
		for (interc_idx = 0;
				interc_idx < pb_type->modes[mode_idx].num_interconnect;
				interc_idx++) {
			power_zero_usage(
					&pb_type->modes[mode_idx].interconnect[interc_idx].power_usage);
		}
	}
}

void power_reset_tile_usage(void) {
	int type_idx;

	for (type_idx = 0; type_idx < num_types; type_idx++) {
		if (type_descriptors[type_idx].pb_type) {
			power_reset_pb_type(type_descriptors[type_idx].pb_type);
		}
	}
}

void power_calc_tile_usage(t_power_usage * cb_power_usage) {
	int x, y, z;
	int type_idx;

	for (type_idx = 0; type_idx < num_types; type_idx++) {
		power_zero_usage(&cb_power_usage[type_idx]);
	}

	power_reset_tile_usage();

	for (x = 0; x < nx + 2; x++) {
		for (y = 0; y < ny + 2; y++) {
			type_idx = grid[x][y].type->index;

			if ((grid[x][y].offset != 0) || (grid[x][y].type == EMPTY_TYPE)) {
				continue;
			}

			for (z = 0; z < grid[x][y].type->capacity; z++) {
				t_pb * pb = NULL;
				t_power_usage pb_power;

				if (grid[x][y].blocks[z] != OPEN) {
					pb = block[grid[x][y].blocks[z]].pb;
				}

				power_calc_pb(&pb_power, pb, grid[x][y].type->pb_graph_head);
				power_add_usage(&cb_power_usage[type_idx], &pb_power);
			}
		}
	}
	return;
}

/*
 * Func Name	: 	calc_power_clock
 *
 * Description	: 	Calculates the total power usage from the clock network
 *
 * Arguments	:
 *
 * Returns		:	Nothing
 */
static void power_calc_clock(t_power_usage * power_usage, t_clocks * clocks) {
	float Clock_power_dissipation;
	int total_clock_buffers, total_clock_segments;
	int clock_idx;

	/* Initialization */
	power_usage->dynamic = 0.;
	power_usage->leakage = 0.;

	Clock_power_dissipation = 0;
	total_clock_buffers = 0;
	total_clock_segments = 0;

	/* fprintf (g_power_output->out_detailed, "\n\n************ Clock Power Analysis ************\n\n"); */

	/* if no global clock, then return */
	if (clocks->num_global_clock == 0) {
		return;
	}

	for (clock_idx = 0; clock_idx < clocks->num_global_clock; clock_idx++) {
		t_power_usage clock_power;

		/* Assume the global clock is active even for combinational circuits */
		if (clocks->num_global_clock == 1) {
			if (clocks->clock_inf[clock_idx].density == 0) {
				clocks->clock_inf[clock_idx].density = 2;
				clocks->clock_inf[clock_idx].prob = 0.5;
			}
		}
		/* find the power dissipated by each clock network */
		power_calc_clock_single(&clock_power, &clocks->clock_inf[clock_idx]);
		power_add_usage(power_usage, &clock_power);
	}

	return;
}

static void power_calc_clock_single(t_power_usage * power_usage,
		t_clock_inf * single_clock) {

	/*
	 *
	 * The following code assumes a spine-and-rib clock network as shown below.
	 * This is comprised of 3 main combonents:
	 * 	1. A single wire from the io pad to the center of the chip
	 * 	2. A H-structure which provides a 'spine' to all 4 quadrants
	 * 	3. Ribs connect each spine with an entire column of blocks

	 ___________________
	 |                 |
	 | |_|_|_2__|_|_|_ |
	 | | | |  | | | |  |
	 | |3| |  | | | |  |
	 |        |        |
	 | | | |  | | | |  |
	 | |_|_|__|_|_|_|_ |
	 | | | |  | | | |  |
	 |_______1|________|
	 * It is assumed that there are a single-inverter buffers placed along each wire,
	 * with spacing equal to the FPGA block size (1 buffer/block) */
	t_power_usage clock_buffer_power;
	boolean clock_used;
	int length;
	t_power_usage buffer_power;
	t_power_usage wire_power;
	float C_segment;
	float buffer_size;

	power_usage->dynamic = 0.;
	power_usage->leakage = 0.;

	/* Currently only a single clock is supported - so it must have some switching */
	assert(single_clock->density);

	/* Check if this clock is active - this is used for calculating leakage */
	if (single_clock->density) {
		clock_used = TRUE;
	} else {
		clock_used = FALSE;
		assert(0);
	}

	C_segment = g_power_common->tile_length * single_clock->C_wire;
	if (single_clock->autosize_buffer) {
		buffer_size = 1 + C_segment / g_power_common->INV_1X_C_in;
	} else {
		buffer_size = single_clock->buffer_size;
	}

	/* Calculate the capacitance and leakage power for the clock buffer */
	power_calc_INV(&clock_buffer_power, single_clock->density,
			single_clock->prob, g_power_arch->P_to_N_size_ratio * buffer_size,
			buffer_size);

	length = 0;

	/* 1. IO to chip center */
	length += (ny + 2) / 2;

	/* 2. H-Tree to 4 quadrants */
	length += ny / 2 + 2 * nx;

	/* 3. Ribs - to */
	length += nx / 2 * ny;

	buffer_power.dynamic = length * clock_buffer_power.dynamic;
	buffer_power.leakage = length * clock_buffer_power.leakage;
	power_add_usage_element(power_usage, &buffer_power,
			POWER_ELEMENT_CLOCK_BUFFER);

	wire_power.dynamic = power_calc_dynamic(length * C_segment,
			single_clock->density);
	wire_power.leakage = 0.;
	power_add_usage_element(power_usage, &wire_power, POWER_ELEMENT_CLOCK_WIRE);

	return;
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

	if (final_stage_size <= 1)
		N = 1;
	else if (final_stage_size < desired_stage_effort)
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

void init_mux_arch_default(t_mux_arch * mux_arch, int levels, int num_inputs) {
	int level_idx;

	mux_arch->levels = levels;
	mux_arch->num_inputs = num_inputs;

	mux_arch_fix_levels(mux_arch);

	mux_arch->encoding_types = my_calloc(mux_arch->levels,
			sizeof(enum e_encoding_type));
	mux_arch->transistor_sizes = my_calloc(mux_arch->levels, sizeof(float));

	for (level_idx = 0; level_idx < mux_arch->levels; level_idx++) {
		mux_arch->encoding_types[level_idx] = ENCODING_DECODER;
		mux_arch->transistor_sizes[level_idx] = 1.0;
	}
	mux_arch->mux_graph_head = alloc_and_load_mux_graph(num_inputs,
			mux_arch->levels);
}

void dealloc_mux_graph_recursive(t_mux_node * node) {
	int child_idx;

	/* Dealloc Children */
	if (node->level != 0) {
		for (child_idx = 0; child_idx < node->num_inputs; child_idx++) {
			dealloc_mux_graph_recursive(&node->children[child_idx]);
		}
		free(node->children);
	}
}

void dealloc_mux_graph(t_mux_node * node) {
	dealloc_mux_graph_recursive(node);
	free(node);
}

void alloc_and_load_mux_graph_recursive(t_mux_node * node,
		int num_primary_inputs, int level, int starting_pin_idx) {
	int child_idx;
	int pin_idx = starting_pin_idx;

	node->num_inputs = pow(num_primary_inputs, 1 / ((float) level + 1)) + 0.5;
	node->level = level;
	node->starting_pin_idx = starting_pin_idx;

	if (level != 0) {
		node->children = my_calloc(node->num_inputs, sizeof(t_mux_node));
		for (child_idx = 0; child_idx < node->num_inputs; child_idx++) {
			int num_child_pi = num_primary_inputs / node->num_inputs;
			if (child_idx < (num_primary_inputs % node->num_inputs)) {
				num_child_pi++;
			}
			alloc_and_load_mux_graph_recursive(&node->children[child_idx],
					num_child_pi, level - 1, pin_idx);
			pin_idx += num_child_pi;
		}
	}
}

t_mux_node *
alloc_and_load_mux_graph(int num_inputs, int levels) {
	t_mux_node * node;

	node = my_malloc(sizeof(t_mux_node));
	alloc_and_load_mux_graph_recursive(node, num_inputs, levels - 1, 0);

	return node;
}

float clb_net_density(int net_idx) {
	if (net_idx == OPEN) {
		return 0.;
	} else {
		return clb_net[net_idx].density;
	}
}

float clb_net_prob(int net_idx) {
	if (net_idx == OPEN) {
		return 0.;
	} else {
		return clb_net[net_idx].probability;
	}
}

static void power_calc_routing(t_power_usage * power_usage) {
	int rr_node_idx;
	int net_idx;
	int edge_idx;

	power_zero_usage(power_usage);

	g_power_common->num_sb_buffers = 0;
	g_power_common->total_sb_buffer_size = 0.;
	g_power_common->num_cb_buffers = 0;
	g_power_common->total_cb_buffer_size = 0.;

	/* Reset rr graph net indices */
	for (rr_node_idx = 0; rr_node_idx < num_rr_nodes; rr_node_idx++) {
		rr_node[rr_node_idx].net_num = OPEN;
	}

	/* Reset Input Density/Prob */
	for (rr_node_idx = 0; rr_node_idx < num_rr_nodes; rr_node_idx++) {
		rr_node[rr_node_idx].num_inputs = 0;
	}

	/* Populate net indices into rr graph */
	for (net_idx = 0; net_idx < num_nets; net_idx++) {
		struct s_trace * trace;

		for (trace = trace_head[net_idx]; trace != NULL; trace = trace->next) {
			rr_node[trace->index].visited = FALSE;
		}
	}

	/* Populate net indices into rr graph */
	for (net_idx = 0; net_idx < num_nets; net_idx++) {
		struct s_trace * trace;

		for (trace = trace_head[net_idx]; trace != NULL; trace = trace->next) {
			t_rr_node * node = &rr_node[trace->index];

			if (node->visited) {
				continue;
			}

			node->net_num = net_idx;

			for (edge_idx = 0; edge_idx < node->num_edges; edge_idx++) {
				if (node->edges[edge_idx] != OPEN) {
					t_rr_node * next_node = &rr_node[node->edges[edge_idx]];

					switch (next_node->type) {
					case CHANX:
					case CHANY:
					case IPIN:
						if (next_node->net_num == node->net_num) {
							next_node->selected_input = next_node->num_inputs;
						}
						next_node->in_density[next_node->num_inputs] =
								clb_net_density(node->net_num);
						next_node->in_prob[next_node->num_inputs] =
								clb_net_prob(node->net_num);
						next_node->num_inputs++;
						if (next_node->num_inputs > next_node->fan_in) {
							printf("%d %d\n", next_node->num_inputs,
									next_node->fan_in);
							fflush(0);
							assert(0);
						}
						break;
					default:
						/* Do nothing */
						break;
					}
				}
			}
			node->visited = TRUE;
		}
	}

	for (rr_node_idx = 0; rr_node_idx < num_rr_nodes; rr_node_idx++) {
		t_power_usage sub_power_usage;
		t_rr_node * node = &rr_node[rr_node_idx];
		float C_relative;
		float C_wire;
		float buffer_size;
		boolean drives_switchbox;
		int switch_idx;
		int connectionbox_fanout;

		switch (node->type) {
		case SOURCE:
		case SINK:
		case OPIN:
			/* No power usage for these types */
			break;
		case IPIN:
			/* This is part of the connectionbox.  The connection box is comprised of:
			 *  - Driver (accounted for at end of CHANX/Y - see below)
			 *  - Multiplexor */

			if (node->fan_in) {
				assert(node->in_density);
				assert(node->in_prob);

				/* Multiplexor */
				power_calc_MUX(&sub_power_usage,
						&g_power_common->mux_arch[node->fan_in],
						node->in_density, node->in_prob, node->selected_input);
				power_add_usage_element(power_usage, &sub_power_usage,
						POWER_ELEMENT_ROUTE_CB);
			}
			break;
		case CHANX:
		case CHANY:
			/* This is a wire driven by a switchbox, which includes:
			 * 	- The Multiplexor at the beginning of the wire
			 * 	- A buffer, after the mux to drive the wire
			 * 	- The wire itself
			 * 	- A buffer at the end of the wire, going to switchbox/connectionbox */
			assert(node->in_density);
			assert(node->in_prob);
			C_wire = node->C_tile_per_m * g_power_common->tile_length;
			assert(node->selected_input < node->fan_in);

			/* Multiplexor */
			power_calc_MUX(&sub_power_usage,
					&g_power_common->mux_arch[node->fan_in], node->in_density,
					node->in_prob, node->selected_input);
			power_add_usage_element(power_usage, &sub_power_usage,
					POWER_ELEMENT_ROUTE_SB);

			/* Buffer Size */
			if (switch_inf[node->driver_switch_type].autosize_buffer) {
				C_relative = (float) node->num_edges
						+ C_wire / g_power_common->INV_1X_C_in;
				buffer_size = C_relative / POWER_BUFFER_STAGE_GAIN;
			} else {
				buffer_size =
						switch_inf[node->driver_switch_type].buffer_last_stage_size;
			}
			buffer_size = max(buffer_size, 1);

			g_power_common->num_sb_buffers++;
			g_power_common->total_sb_buffer_size += buffer_size;

			/* Buffer */
			power_calc_buffer(&sub_power_usage, buffer_size,
					node->in_density[node->selected_input],
					node->in_prob[node->selected_input]);
			power_add_usage_element(power_usage, &sub_power_usage,
					POWER_ELEMENT_ROUTE_SB);

			/* Wire Capacitance */
			sub_power_usage.dynamic = power_calc_dynamic(C_wire,
					clb_net_density(node->net_num));
			sub_power_usage.leakage = 0.;
			power_add_usage_element(power_usage, &sub_power_usage,
					POWER_ELEMENT_ROUTE_GLB_WIRE);

			/* Determine types of switches that this wire drives */
			connectionbox_fanout = 0;
			drives_switchbox = FALSE;
			for (switch_idx = 0; switch_idx < node->num_edges; switch_idx++) {
				if (node->switches[switch_idx]
						== g_routing_arch->wire_to_ipin_switch) {
					connectionbox_fanout++;
				} else if (node->switches[switch_idx]
						== g_routing_arch->delayless_switch) {
					/* Do nothing */
				} else {
					drives_switchbox = TRUE;
				}
			}

			/* Buffer to next Switchbox */
			if (drives_switchbox) {
				power_calc_INV1(&sub_power_usage,
						node->in_density[node->selected_input],
						1 - node->in_prob[node->selected_input]);
				power_add_usage_element(power_usage, &sub_power_usage,
						POWER_ELEMENT_ROUTE_SB);
			}

			/* Driver for ConnectionBox */
			if (connectionbox_fanout) {
				float connectionbox_effort = connectionbox_fanout
						* (g_power_common->NMOS_1X_C_drain_avg
								/ g_power_common->INV_1X_C_in);
				buffer_size =
						max(1, connectionbox_effort / POWER_BUFFER_STAGE_GAIN);
				power_calc_buffer(&sub_power_usage, buffer_size,
						node->in_density[node->selected_input],
						1 - node->in_prob[node->selected_input]);
				power_add_usage_element(power_usage, &sub_power_usage,
						POWER_ELEMENT_ROUTE_CB);

				g_power_common->num_cb_buffers++;
				g_power_common->total_cb_buffer_size += buffer_size;
			}
			break;
		case INTRA_CLUSTER_EDGE:
			assert(0);
			break;
		default:
			power_log_msg(POWER_LOG_WARNING,
					"The global routing-resource graph contains an unknown node type.");
			break;
		}
	}
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

void power_calc_buffer(t_power_usage * power_usage, float size, float in_dens,
		float in_prob) {
	t_power_usage sub_power_usage;
	int i, num_stages;
	float stage_effort;
	float stage_inv_size;
	float stage_in_prob;

	power_zero_usage(power_usage);

	if (size == 0) {
		return;
	}

	num_stages = calc_buffer_num_stages(size, POWER_BUFFER_STAGE_GAIN);
	stage_effort = calc_buffer_stage_effort(num_stages, size);

	stage_in_prob = in_prob;
	for (i = 0; i < num_stages; i++) {
		stage_inv_size = pow(stage_effort, i);

		if (i == 0) {
			/* Sense Buffer */
			power_calc_level_restorer(&sub_power_usage, in_dens, stage_in_prob);
			/*power_calc_INV(&sub_power_usage, in_dens, stage_in_prob, 1, 2);*/
		} else {
			power_calc_INV(&sub_power_usage, in_dens, stage_in_prob,
					stage_inv_size * g_power_arch->P_to_N_size_ratio,
					stage_inv_size);
		}
		power_add_usage(power_usage, &sub_power_usage);

		stage_in_prob = 1 - stage_in_prob;
	}
}

#if 0
char *
str_first_not_in(char * str, char * delimiters) {
	int i = strcspn(str, delimiters);

	return str + i;
}

float get_tx_param(char * tech_data, char * key) {
	char * pos;
	char * pos_end;
	int key_len;
	char buffer[30];
	char msg[256];

	pos = tech_data;

	while ((pos = strstr(tech_data, key))) {
		key_len = strcspn(pos, cmos_tech_delim);

		/* check if the keys match, as some keys maybe be substrings of others */
		if (strncmp(key, pos, key_len) == 0) {
			/* Key is found */

			/* Find '=' */
			pos = strchr(pos + key_len, '=');
			pos++;

			/* Find Start of value */
			pos += strspn(pos, cmos_tech_delim);

			/* Find End of value */
			pos_end = strpbrk(pos, cmos_tech_delim);

			strncpy(buffer, pos, pos_end - pos);
			buffer[pos_end - pos] = '\0';

			return atof(buffer);
		}
	}

	sprintf(msg, "CMOS technology file does not contain a value for '%s'", key);
	power_log_msg(POWER_LOG_ERROR, msg);

	return 0;
}
#endif

void process_transistor_info(ezxml_t parent) {
	t_transistor_inf * trans_inf;
	const char * prop;
	ezxml_t child, prev, grandchild;
	int i;

	prop = FindProperty(parent, "type", TRUE);
	if (strcmp(prop, "nmos") == 0) {
		trans_inf = &g_power_arch->NMOS_tx_record;
	} else if (strcmp(prop, "pmos") == 0) {
		trans_inf = &g_power_arch->PMOS_tx_record;
	} else {
		assert(0);
	}
	ezxml_set_attr(parent, "type", NULL);

	trans_inf->Vth = GetFloatProperty(parent, "Vth", FALSE, 0.);

	trans_inf->long_trans_inf = my_malloc(sizeof(t_transistor_size_inf));

	child = FindElement(parent, "long_size", TRUE);
	assert(GetIntProperty(child, "L", TRUE, 0) == 2);
	trans_inf->long_trans_inf->size = GetFloatProperty(child, "W", TRUE, 0);

	grandchild = FindElement(child, "current", TRUE);
	trans_inf->long_trans_inf->leakage = GetFloatProperty(grandchild, "leakage",
			TRUE, 0);
	FreeNode(grandchild);

	grandchild = FindElement(child, "capacitance", TRUE);
	trans_inf->long_trans_inf->C_gate_cmos = GetFloatProperty(grandchild,
			"gate_cmos", TRUE, 0);
	/*trans_inf->long_trans_inf->C_source_cmos = GetFloatProperty(grandchild,
	 "source_cmos", TRUE, 0);*/
	trans_inf->long_trans_inf->C_drain_cmos = GetFloatProperty(grandchild,
			"drain_cmos", TRUE, 0);
	trans_inf->long_trans_inf->C_gate_pass = GetFloatProperty(grandchild,
			"gate_pass", TRUE, 0);
	trans_inf->long_trans_inf->C_source_pass = GetFloatProperty(grandchild,
			"source_pass", TRUE, 0);
	trans_inf->long_trans_inf->C_drain_pass = GetFloatProperty(grandchild,
			"drain_pass", TRUE, 0);
	FreeNode(grandchild);

	trans_inf->num_size_entries = CountChildren(parent, "size", 1);
	trans_inf->size_inf = my_calloc(trans_inf->num_size_entries,
			sizeof(t_transistor_size_inf));
	FreeNode(child);

	child = FindFirstElement(parent, "size", TRUE);
	i = 0;
	while (child) {
		assert(GetIntProperty(child, "L", TRUE, 0) == 1);

		trans_inf->size_inf[i].size = GetFloatProperty(child, "W", TRUE, 0);

		grandchild = FindElement(child, "current", TRUE);
		trans_inf->size_inf[i].leakage = GetFloatProperty(grandchild, "leakage",
				TRUE, 0);
		FreeNode(grandchild);

		grandchild = FindElement(child, "capacitance", TRUE);
		trans_inf->size_inf[i].C_gate_cmos = GetFloatProperty(grandchild,
				"gate_cmos", TRUE, 0);
		/*trans_inf->size_inf[i].C_source_cmos = GetFloatProperty(grandchild,
		 "source_cmos", TRUE, 0); */
		trans_inf->size_inf[i].C_drain_cmos = GetFloatProperty(grandchild,
				"drain_cmos", TRUE, 0);
		trans_inf->size_inf[i].C_gate_pass = GetFloatProperty(grandchild,
				"gate_pass", TRUE, 0);
		trans_inf->size_inf[i].C_source_pass = GetFloatProperty(grandchild,
				"source_pass", TRUE, 0);
		trans_inf->size_inf[i].C_drain_pass = GetFloatProperty(grandchild,
				"drain_pass", TRUE, 0);
		FreeNode(grandchild);

		prev = child;
		child = child->next;
		FreeNode(prev);
		i++;
	}
}

boolean file_exists(const char * filename) {
	FILE * file;

	if (filename == NULL) {
		return FALSE;
	}

	file = fopen(filename, "r");
	if (file) {
		fclose(file);
		return TRUE;
	}
	return FALSE;
}

void power_read_cmos_tech_behavior(void) {
	ezxml_t cur, child, prev;
	const char * prop;
	char msg[BUFSIZE];

	if (!file_exists(g_power_opts->cmos_tech_behavior_file)) {
		sprintf(msg,
				"The CMOS technology behavior file ('%s') does not exist.  No power information will be calculated.",
				g_power_opts->cmos_tech_behavior_file);
		power_log_msg(POWER_LOG_ERROR, msg);
		g_power_arch->NMOS_tx_record.num_size_entries = 0;
		g_power_arch->NMOS_tx_record.long_trans_inf = NULL;
		g_power_arch->NMOS_tx_record.size_inf = NULL;
		g_power_arch->NMOS_tx_record.Vth = 0.;

		g_power_arch->PMOS_tx_record.num_size_entries = 0;
		g_power_arch->PMOS_tx_record.long_trans_inf = NULL;
		g_power_arch->PMOS_tx_record.size_inf = NULL;
		g_power_arch->PMOS_tx_record.Vth = 0.;

		g_power_arch->Vdd = 0.;
		g_power_arch->temperature = 27;
		g_power_arch->P_to_N_size_ratio = 1.;
		return;
	}
	cur = ezxml_parse_file(g_power_opts->cmos_tech_behavior_file);

	prop = FindProperty(cur, "file", TRUE);
	ezxml_set_attr(cur, "file", NULL);

	prop = FindProperty(cur, "size", TRUE);
	g_power_arch->tech_size = atof(prop);
	ezxml_set_attr(cur, "size", NULL);

	child = FindElement(cur, "operating_point", TRUE);
	g_power_arch->temperature = GetFloatProperty(child, "temperature", TRUE, 0);
	g_power_arch->Vdd = GetFloatProperty(child, "Vdd", TRUE, 0);
	FreeNode(child);

	child = FindElement(cur, "p_to_n", TRUE);
	g_power_arch->P_to_N_size_ratio = GetFloatProperty(child, "ratio", TRUE, 0);
	FreeNode(child);

	child = FindFirstElement(cur, "transistor", TRUE);
	process_transistor_info(child);

	prev = child;
	child = child->next;
	FreeNode(prev);

	process_transistor_info(child);
	FreeNode(child);

	FreeNode(cur);
}

/*
 * Func Name	: 	power_init
 *
 * Description	: 	Initialization for all power-related functions
 *
 * Arguments	: 	power_otions
 * 					power_architecture
 *
 * Returns		: 	Nothing
 */
boolean power_init(t_power_opts * power_opts, t_power_arch * power_arch) {
	boolean error = FALSE;
	int net_idx;
	float C_gate, C_source, C_drain;
	int rr_node_idx;
	int max_fanin;
	int max_IPIN_fanin;
	int max_seg_to_IPIN_fanout;
	int max_seg_to_seg_fanout;
	int max_seg_fanout;
	int mux_size;
	int i;
	float transistors_per_tile;

	/* Set global power architecture & options */
	g_power_opts = power_opts;
	g_power_arch = power_arch;
	g_power_common = my_malloc(sizeof(t_power_commonly_used));
	g_power_output = my_malloc(sizeof(t_power_output));

	/* Set up Logs */
	g_power_output->num_logs = POWER_LOG_NUM_TYPES;
	g_power_output->logs = my_calloc(g_power_output->num_logs, sizeof(t_log));
	g_power_output->logs[POWER_LOG_ERROR].name = my_strdup("Errors");
	g_power_output->logs[POWER_LOG_WARNING].name = my_strdup("Warnings");

	/* Initialize output file */
	if (!error) {
		g_power_output->out = NULL;
		g_power_output->out = my_fopen(g_power_opts->power_output_file, "w", 0);
		if (!g_power_output->out) {
			error = TRUE;
		}
	}

	power_read_cmos_tech_behavior();

	/* Track power for all types */
	g_power_breakdown = my_malloc(sizeof(t_power_breakdown));
	g_power_breakdown->elements = my_calloc(POWER_ELEMENT_MAX_NUM,
			sizeof(t_power_usage));
	for (i = 0; i < POWER_ELEMENT_MAX_NUM; i++) {
		power_zero_usage(&g_power_breakdown->elements[i]);
	}

	/* Initialize Commonly Used Values */
	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS, 1.0,
			POWER_CIRCUIT_TYPE_CMOS);
	g_power_common->NMOS_1X_C_drain_max = C_drain;
	g_power_common->NMOS_1X_C_gate_max = C_gate;
	g_power_common->NMOS_1X_C_source_max = C_source;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS, 1.0,
			POWER_CIRCUIT_TYPE_PASS);
	g_power_common->NMOS_1X_C_drain_avg = C_drain;
	g_power_common->NMOS_1X_C_gate_avg = C_gate;
	g_power_common->NMOS_1X_C_source_avg = C_source;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS,
			power_arch->P_to_N_size_ratio, POWER_CIRCUIT_TYPE_CMOS);
	g_power_common->PMOS_1X_C_drain_max = C_drain;
	g_power_common->PMOS_1X_C_gate_max = C_gate;
	g_power_common->PMOS_1X_C_source_max = C_source;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS,
			power_arch->P_to_N_size_ratio, POWER_CIRCUIT_TYPE_PASS);
	g_power_common->PMOS_1X_C_drain_avg = C_drain;
	g_power_common->PMOS_1X_C_gate_avg = C_gate;
	g_power_common->PMOS_1X_C_source_avg = C_source;

	g_power_common->NMOS_1X_leakage = power_calc_transistor_leakage(NMOS, 1.0);
	g_power_common->PMOS_1X_leakage = power_calc_transistor_leakage(PMOS,
			1.0 * g_power_arch->P_to_N_size_ratio);
	g_power_common->NMOS_2X_leakage = power_calc_transistor_leakage(NMOS, 2.0);
	g_power_common->PMOS_2X_leakage = power_calc_transistor_leakage(PMOS,
			2.0 * g_power_arch->P_to_N_size_ratio);

	g_power_common->INV_1X_C_in = g_power_common->NMOS_1X_C_gate_max
			+ g_power_common->PMOS_1X_C_gate_max;
	g_power_common->INV_1X_C = g_power_common->NMOS_1X_C_gate_max
			+ g_power_common->PMOS_1X_C_gate_max
			+ g_power_common->NMOS_1X_C_drain_max
			+ g_power_common->PMOS_1X_C_drain_max;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS, 2.0,
			POWER_CIRCUIT_TYPE_CMOS);
	g_power_common->INV_2X_C = C_gate + C_drain;
	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, PMOS,
			2.0 * g_power_arch->P_to_N_size_ratio, POWER_CIRCUIT_TYPE_CMOS);
	g_power_common->INV_2X_C = C_gate + C_drain;

	g_power_common->SRAM_bit_size = power_SRAM_bit_size();

	/* Copy probability/density values to new netlist */
	for (net_idx = 0; net_idx < num_nets; net_idx++) {
		clb_net[net_idx].probability =
				vpack_net[clb_to_vpack_net_mapping[net_idx]].probability;
		clb_net[net_idx].density =
				vpack_net[clb_to_vpack_net_mapping[net_idx]].density;
	}

	/* Initialize Mux Architectures */
	max_fanin = 0;
	max_IPIN_fanin = 0;
	max_seg_to_seg_fanout = 0;
	max_seg_to_IPIN_fanout = 0;
	for (rr_node_idx = 0; rr_node_idx < num_rr_nodes; rr_node_idx++) {
		int switch_idx;
		int fanout_to_IPIN = 0;
		int fanout_to_seg = 0;
		t_rr_node * node = &rr_node[rr_node_idx];

		node->driver_switch_type = OPEN;

		switch (node->type) {
		case IPIN:
			max_IPIN_fanin = max(max_IPIN_fanin, node->fan_in);
			max_fanin = max(max_fanin, node->fan_in);

			node->in_density = my_calloc(node->fan_in, sizeof(float));
			node->in_prob = my_calloc(node->fan_in, sizeof(float));
			break;
		case CHANX:
		case CHANY:
			for (switch_idx = 0; switch_idx < node->num_edges; switch_idx++) {
				if (node->switches[switch_idx]
						== g_routing_arch->wire_to_ipin_switch) {
					fanout_to_IPIN++;
				} else if (node->switches[switch_idx]
						!= g_routing_arch->delayless_switch) {
					fanout_to_seg++;
				}
			}
			max_seg_to_IPIN_fanout = max(max_seg_to_IPIN_fanout, fanout_to_IPIN);
			max_seg_to_seg_fanout = max(max_seg_to_seg_fanout, fanout_to_seg);
			max_fanin = max(max_fanin, node->fan_in);

			node->in_density = my_calloc(node->fan_in, sizeof(float));
			node->in_prob = my_calloc(node->fan_in, sizeof(float));
			break;
		default:
			/* Do nothing */
			break;
		}
	}
	g_power_common->max_mux_size = max_fanin;
	g_power_common->max_IPIN_fanin = max_IPIN_fanin;
	g_power_common->max_seg_to_seg_fanout = max_seg_to_seg_fanout;
	g_power_common->max_seg_to_IPIN_fanout = max_seg_to_IPIN_fanout;

	if (PRINT_SPICE_COMPARISON) {
		g_power_common->max_mux_size = max(g_power_common->max_mux_size, 20);
	}

	/* Set up mux architectures for switch/connection boxes up to max size */
	g_power_common->mux_arch = my_calloc(g_power_common->max_mux_size + 1,
			sizeof(t_mux_arch));
	for (mux_size = 1; mux_size <= g_power_common->max_mux_size; mux_size++) {
		init_mux_arch_default(&g_power_common->mux_arch[mux_size], 2, mux_size);
	}

	/* Populate driver switch type */
	for (rr_node_idx = 0; rr_node_idx < num_rr_nodes; rr_node_idx++) {
		t_rr_node * node = &rr_node[rr_node_idx];
		int edge_idx;

		for (edge_idx = 0; edge_idx < node->num_edges; edge_idx++) {
			if (node->edges[edge_idx] != OPEN) {
				if (rr_node[node->edges[edge_idx]].driver_switch_type == OPEN) {
					rr_node[node->edges[edge_idx]].driver_switch_type =
							node->switches[edge_idx];
				} else {
					assert(
							rr_node[node->edges[edge_idx]].driver_switch_type == node->switches[edge_idx]);
				}
			}
		}
	}

	/* Find Max Fanout of Routing Buffer	 */
	max_seg_fanout = 0;
	for (rr_node_idx = 0; rr_node_idx < num_rr_nodes; rr_node_idx++) {
		t_rr_node * node = &rr_node[rr_node_idx];

		switch (node->type) {
		case CHANX:
		case CHANY:
			if (node->num_edges > max_seg_fanout) {
				max_seg_fanout = node->num_edges;
			}
			break;
		default:
			/* Do nothing */
			break;
		}
	}
	g_power_common->max_seg_fanout = max_seg_fanout;

	/* Find # of transistors in each tile type */
	transistors_per_tile = power_count_transistors_tiles();

	/* Calculate CLB tile size
	 *  - Assume a square tile
	 *  - Assume min transistor size is Wx6L
	 *  - Assume an overhead to space transistors
	 */
	g_power_common->tile_length = sqrt(
			power_transistor_area(transistors_per_tile));

	return error;
}

static float power_transistor_area(float num_transistors) {
	/* Calculate transistor area, assuming:
	 *  - Min transistor size is Wx6L
	 *  - Overhead to space transistors is: POWER_TRANSISTOR_SPACING_FACTOR */

	return num_transistors * POWER_TRANSISTOR_SPACING_FACTOR
			* (g_power_arch->tech_size * g_power_arch->tech_size * 6);
}

static float power_SRAM_bit_size(void) {
	/* The SRAM cell consists of:
	 *  - 2 weak PMOS for pullup (size 1)
	 *  - 2 strong NMOS for pulldown (size 4)
	 *  - 2 moderate NMOS for access (size 2)
	 *
	 *  This information is from Weste/Harris "CMOS VLSI Design"
	 */

	return (1 + 1 + 4 + 4 + 2 + 2);
}

static float power_count_transistors_LUT(int LUT_size) {
	float transistor_cnt;
	int level_idx;

	/* Each input driver has 1-1X and 2-2X inverters */
	transistor_cnt = (float) LUT_size
			* ((1 + g_power_arch->P_to_N_size_ratio)
					+ 2 * (2 + 2 * g_power_arch->P_to_N_size_ratio));
	/*transistor_cnt = (float) LUT_size
	 * ((1 + g_power_arch->P_to_N_size_ratio)
	 + 2 * (2 + 2 * g_power_arch->P_to_N_size_ratio));*/

	/* SRAM bits */
	transistor_cnt += g_power_common->SRAM_bit_size * pow(2, LUT_size);

	for (level_idx = 0; level_idx < LUT_size; level_idx++) {

		/* Pass transistors */
		transistor_cnt += pow(2, LUT_size - level_idx);

		/* Add level restorer after every 2 stages (level_idx %2 == 1)
		 * But if there is an odd # of stages, just put one at the last
		 * stage (level_idx == LUT_size - 1) and not at the stage just before
		 * the last stage (level_idx != LUT_size - 2)
		 */
		if (((level_idx % 2 == 1) && (level_idx != LUT_size - 2))
				|| (level_idx == LUT_size - 1)) {
			/* Each level restorer has a P/N=1/2 inverter and a W/L=1/2 PMOS */
			transistor_cnt += pow(2, LUT_size - level_idx - 1) * (3 + 2);
		}
	}

	return transistor_cnt;
}

static float power_count_transistors_FF(void) {
	float transistor_cnt = 0.;

	/* 4 1X Inverters */
	transistor_cnt += 4 * (1. + g_power_arch->P_to_N_size_ratio);

	/* 2 Muxes = 4 transmission gates */
	transistor_cnt += 4 * (1. + g_power_arch->P_to_N_size_ratio);

	return transistor_cnt;
}

static float power_count_transistors_blif_primitive(t_pb_type * pb_type) {
	float transistor_cnt;

	if (strcmp(pb_type->blif_model, ".names") == 0) {
		/* LUT */
		transistor_cnt = power_count_transistors_LUT(pb_type->num_input_pins);
	} else if (strcmp(pb_type->blif_model, ".latch") == 0) {
		/* Latch */
		transistor_cnt = power_count_transistors_FF();
	} else {
		/* Other */
		char msg[BUFSIZE];

		sprintf(msg, "No transistor counter function for BLIF model: %s",
				pb_type->blif_model);
		power_log_msg(POWER_LOG_WARNING, msg);
		transistor_cnt = 0;
	}

	return transistor_cnt;
}

static float power_count_transistors_MUX_node(t_mux_node * mux_node,
		float * transistor_sizes) {
	int input_idx;
	float transistor_cnt = 0.;

	if (mux_node->num_inputs != 1) {
		for (input_idx = 0; input_idx < mux_node->num_inputs; input_idx++) {
			/* Single Pass transistor */
			transistor_cnt += transistor_sizes[mux_node->level];

			/* Child MUX */
			if (mux_node->level != 0) {
				transistor_cnt += power_count_transistors_MUX_node(
						&mux_node->children[input_idx], transistor_sizes);
			}
		}
	}

	return transistor_cnt;
}

static void power_mux_node_max_inputs(t_mux_node * mux_node, float * max_inputs) {

	max_inputs[mux_node->level] =
			max(max_inputs[mux_node->level], mux_node->num_inputs);

	if (mux_node->level != 0) {
		int child_idx;

		for (child_idx = 0; child_idx < mux_node->num_inputs; child_idx++) {
			power_mux_node_max_inputs(&mux_node->children[child_idx],
					max_inputs);
		}
	}
}

static float power_count_transistors_MUX(t_mux_arch * mux_arch) {
	float transistor_cnt = 0.;
	int lvl_idx;
	float * max_inputs;

	/* SRAM bits */
	max_inputs = my_calloc(mux_arch->levels, sizeof(float));
	for (lvl_idx = 0; lvl_idx < mux_arch->levels; lvl_idx++) {
		max_inputs[lvl_idx] = 0.;
	}
	power_mux_node_max_inputs(mux_arch->mux_graph_head, max_inputs);

	for (lvl_idx = 0; lvl_idx < mux_arch->levels; lvl_idx++) {
		if (mux_arch->encoding_types[lvl_idx] == ENCODING_DECODER) {
			transistor_cnt += ceil(log2((float)max_inputs[lvl_idx]))
					* g_power_common->SRAM_bit_size;
			/* TODO_POWER - Size of decoder */
		} else if (mux_arch->encoding_types[lvl_idx] == ENCODING_ONE_HOT) {
			transistor_cnt += max_inputs[lvl_idx]
					* g_power_common->SRAM_bit_size;
		} else {
			assert(0);
		}
	}

	transistor_cnt += power_count_transistors_MUX_node(mux_arch->mux_graph_head,
			mux_arch->transistor_sizes);

	return transistor_cnt;
}

static float power_count_transistors_interconnect(t_interconnect * interc) {
	float transistor_cnt = 0.;

	switch (interc->type) {
	case DIRECT_INTERC:
		/* No transistors */
		break;
	case MUX_INTERC:
	case COMPLETE_INTERC:
		/* Bus based interconnect:
		 * - Each output port requires a (num_input_ports:1) bus-based multiplexor.
		 * - The number of muxes required for bus based multiplexors is equivalent to
		 * the width of the bus (num_pins_per_port).
		 */
		transistor_cnt += interc->num_output_ports * interc->num_pins_per_port
				* power_count_transistors_MUX(interc->mux_arch);
		break;
	default:
		assert(0);
	}

	return transistor_cnt;
}

static float power_count_transistors_pb_type(t_pb_type * pb_type) {
	int mode_idx;
	float transistor_cnt_max = 0;
	float transistor_cnt_interc = 0;
	boolean ignore_interc = FALSE;

	/* Check if this is a leaf node, or whether it has children */
	if (pb_type->num_modes == 0) {
		/* Leaf node */
		transistor_cnt_interc = 0;
		transistor_cnt_max = power_count_transistors_blif_primitive(pb_type);
	} else {
		/* Find max transistor count between all modes */
		for (mode_idx = 0; mode_idx < pb_type->num_modes; mode_idx++) {
			int interc_idx;
			int child_type_idx;
			float transistor_cnt_mode;
			float transistor_cnt_children = 0;
			float transistor_cnt_mode_interc = 0;

			if (pb_type->class_type == LUT_CLASS) {
				/* LUTs will have a child node that is used for routing purposes
				 * For the routing algorithms it is completely connected; however,
				 * this interconnect does not exist in FPGA hardware and should
				 * be ignored for power calculations. */
				ignore_interc = TRUE;
			}

			/* Count Interconnect Transistors */
			if (!ignore_interc) {
				for (interc_idx = 0;
						interc_idx < pb_type->modes[mode_idx].num_interconnect;
						interc_idx++) {
					transistor_cnt_mode_interc +=
							power_count_transistors_interconnect(
									&pb_type->modes[mode_idx].interconnect[interc_idx]);
				}
			}

			/* Count Child Transistors */
			for (child_type_idx = 0;
					child_type_idx
							< pb_type->modes[mode_idx].num_pb_type_children;
					child_type_idx++) {
				t_pb_type * child_type =
						&pb_type->modes[mode_idx].pb_type_children[child_type_idx];
				transistor_cnt_children += child_type->num_pb
						* power_count_transistors_pb_type(child_type);
			}

			transistor_cnt_mode = (transistor_cnt_mode_interc
					+ transistor_cnt_children);

			if (transistor_cnt_mode > transistor_cnt_max) {
				transistor_cnt_max = transistor_cnt_mode;
				transistor_cnt_interc = transistor_cnt_mode_interc;
			}
		}
	}

	pb_type->transistor_cnt = transistor_cnt_max;
	pb_type->transistor_cnt_interc = transistor_cnt_interc;

	return transistor_cnt_max;
}

static float power_count_transistors_connectionbox(void) {
	float transistor_cnt = 0.;
	int CLB_inputs;
	float buffer_size;

	assert(FILL_TYPE->pb_graph_head->num_input_ports == 1);
	CLB_inputs = FILL_TYPE->pb_graph_head->num_input_pins[0];

	/* Buffers from Tracks */
	buffer_size =
			g_power_common->max_seg_to_IPIN_fanout
					* (g_power_common->NMOS_1X_C_drain_avg
							/ g_power_common->INV_1X_C_in)/ POWER_BUFFER_STAGE_GAIN;
	buffer_size = max(1, buffer_size);
	transistor_cnt += g_solution_inf->channel_width
			* power_count_transistors_buffer(buffer_size);

	/* Muxes to IPINs */
	transistor_cnt += CLB_inputs
			* power_count_transistors_MUX(
					&g_power_common->mux_arch[g_power_common->max_IPIN_fanin]);

	return transistor_cnt;
}

static float power_count_transistors_buffer(float buffer_size) {
	int stages;
	float effort;
	float stage_size;
	int stage_idx;
	float transistor_cnt = 0.;

	stages = calc_buffer_num_stages(buffer_size, POWER_BUFFER_STAGE_GAIN);
	effort = calc_buffer_stage_effort(stages, buffer_size);

	stage_size = 1;
	for (stage_idx = 0; stage_idx < stages; stage_idx++) {
		transistor_cnt += stage_size * (1 + g_power_arch->P_to_N_size_ratio);
		stage_size *= effort;
	}

	return transistor_cnt;
}

static float power_count_transistors_switchbox(void) {
	float transistor_cnt = 0.;
	float transistors_per_buf_mux = 0.;
	int seg_idx;

	/* Buffer */
	transistors_per_buf_mux += power_count_transistors_buffer(
			(float) g_power_common->max_seg_fanout / POWER_BUFFER_STAGE_GAIN);

	/* Multiplexor */
	transistors_per_buf_mux += power_count_transistors_MUX(
			&g_power_common->mux_arch[g_power_common->max_mux_size]);

	for (seg_idx = 0; seg_idx < g_arch->num_segments; seg_idx++) {
		/* In each switchbox, the different types of segments occur with relative freqencies.
		 * Thus the total number of wires of each segment type is (#tracks * freq * 2).
		 * The (x2) factor accounts for vertical and horizontal tracks.
		 * Of the wires of each segment type only (1/seglength) will have a mux&buffer.
		 */
		float freq_frac = (float) g_arch->Segments[seg_idx].frequency
				/ (float) MAX_CHANNEL_WIDTH;

		transistor_cnt += transistors_per_buf_mux * 2 * freq_frac
				* g_solution_inf->channel_width
				* (1 / (float) g_arch->Segments[seg_idx].length);
	}

	return transistor_cnt;
}

static float power_count_transistors_tiles(void) {
	int type_idx;
	float transistor_cnt = 0.;

	for (type_idx = 0; type_idx < num_types; type_idx++) {
		if (type_descriptors[type_idx].pb_type) {
			power_count_transistors_pb_type(type_descriptors[type_idx].pb_type);
		}
	}

	transistor_cnt += FILL_TYPE->pb_type->transistor_cnt;

	transistor_cnt += 2 * power_count_transistors_switchbox();

	transistor_cnt += 2 * power_count_transistors_connectionbox();

	return transistor_cnt;
}

boolean power_uninit(void) {
	int mux_size;
	int log_idx;
	int rr_node_idx;
	int msg_idx;
	boolean error = FALSE;

	for (rr_node_idx = 0; rr_node_idx < num_rr_nodes; rr_node_idx++) {
		t_rr_node * node = &rr_node[rr_node_idx];

		switch (node->type) {
		case CHANX:
		case CHANY:
		case IPIN:
			if (node->fan_in) {
				free(node->in_density);
				free(node->in_prob);
			}
			break;
		default:
			/* Do nothing */
			break;
		}
	}

	for (mux_size = 1; mux_size <= g_power_common->max_mux_size; mux_size++) {
		free(g_power_common->mux_arch[mux_size].encoding_types);
		free(g_power_common->mux_arch[mux_size].transistor_sizes);
		dealloc_mux_graph(g_power_common->mux_arch[mux_size].mux_graph_head);
	}
	free(g_power_common->mux_arch);
	free(g_power_common);

	if (g_power_output->out) {
		fclose(g_power_output->out);
	}
	for (log_idx = 0; log_idx < g_power_output->num_logs; log_idx++) {
		for (msg_idx = 0; msg_idx < g_power_output->logs[log_idx].num_messages;
				msg_idx++) {
			free(g_power_output->logs[log_idx].messages[msg_idx]);
		}
		free(g_power_output->logs[log_idx].messages);
		free(g_power_output->logs[log_idx].name);
	}
	free(g_power_output->logs);
	free(g_power_output);

	free(g_power_breakdown->elements);
	free(g_power_breakdown);

	return error;
}

void power_print_element_usage(char * name, e_power_element_type type,
		e_power_element_type parent_type, int indent_level) {
	int i;

	const int component_str_size = 30;

	if (type == POWER_ELEMENT_TOTAL) {
		fprintf(g_power_output->out, "%-*s%-20s%-15s%-15s%-15s\n",
				component_str_size, "Component", "Power (W)", "%-Parent",
				"%-Total", "%-Dynamic");
	}

	for (i = 0; i < indent_level; i++) {
		fprintf(g_power_output->out, "  ");
	}

	fprintf(g_power_output->out, "%-*s%-20.4g%-15.3g%-15.3g%-15.3g\n",
			component_str_size - 2 * indent_level, name,
			power_usage_combined(&g_power_breakdown->elements[type]),
			power_usage_combined(&g_power_breakdown->elements[type])
					/ power_usage_combined(
							&g_power_breakdown->elements[parent_type]),
			power_usage_combined(&g_power_breakdown->elements[type])
					/ power_usage_combined(
							&g_power_breakdown->elements[POWER_ELEMENT_TOTAL]),
			g_power_breakdown->elements[type].dynamic
					/ power_usage_combined(&g_power_breakdown->elements[type]));

	switch (type) {
	case (POWER_ELEMENT_TOTAL):
		power_print_element_usage("Routing", POWER_ELEMENT_ROUTING, type,
				indent_level + 1);
		power_print_element_usage("Clock", POWER_ELEMENT_CLOCK, type,
				indent_level + 1);
		power_print_element_usage("Tiles", POWER_ELEMENT_TILES, type,
				indent_level + 1);
		break;
	case (POWER_ELEMENT_ROUTING):
		power_print_element_usage("Switchbox", POWER_ELEMENT_ROUTE_SB, type,
				indent_level + 1);
		power_print_element_usage("Connectionbox", POWER_ELEMENT_ROUTE_CB, type,
				indent_level + 1);
		power_print_element_usage("GlobalWires", POWER_ELEMENT_ROUTE_GLB_WIRE,
				type, indent_level + 1);
		break;
	case (POWER_ELEMENT_CLOCK):
		power_print_element_usage("ClockBuffer", POWER_ELEMENT_CLOCK_BUFFER,
				type, indent_level + 1);
		power_print_element_usage("ClockWire", POWER_ELEMENT_CLOCK_WIRE, type,
				indent_level + 1);
		break;
	case (POWER_ELEMENT_TILES):
		power_print_element_usage("Interconnect", POWER_ELEMENT_LOCAL_INTERC,
				type, indent_level + 1);
		power_print_element_usage("LocalWire", POWER_ELEMENT_LOCAL_WIRE, type,
				indent_level + 1);
		power_print_element_usage("FF", POWER_ELEMENT_FF, type,
				indent_level + 1);
		power_print_element_usage("LUT", POWER_ELEMENT_LUT, type,
				indent_level + 1);
		break;
	case (POWER_ELEMENT_LUT):
		/*
		 power_print_element_usage("Driver", POWER_ELEMENT_LUT_DRIVER, type,
		 indent_level + 1);
		 power_print_element_usage("Mux", POWER_ELEMENT_LUT_MUX, type,
		 indent_level + 1);
		 power_print_element_usage("Restorer", POWER_ELEMENT_LUT_RESTORER, type,
		 indent_level + 1);
		 */
		break;
	default:
		break;
	}

	if (type == POWER_ELEMENT_TOTAL) {
		fprintf(g_power_output->out, "\n");
	}
}

void power_print_element_usage_close(char * name, int indent_level) {
	int i;
	for (i = 0; i < indent_level; i++) {
		fprintf(g_power_output->out, "\t");
	}
	fprintf(g_power_output->out, "</%s>\n", name);
}

char * interconnect_type_name(enum e_interconnect type) {
	switch (type) {
	case COMPLETE_INTERC:
		return "complete";
	case MUX_INTERC:
		return "mux";
	case DIRECT_INTERC:
		return "direct";
	default:
		return "";
	}
}

void power_print_pb_usage_recursive(FILE * fp, t_pb_type * type,
		int indent_level, float parent_power, float total_power) {
	int mode_idx;
	int mode_indent;
	int child_idx;
	int interc_idx;
	float pb_type_power;

	pb_type_power = type->power_usage.dynamic + type->power_usage.leakage;

	print_tabs(fp, indent_level);
	fprintf(fp,
			"<pb_type name=\"%s\" P=\"%.4g\" P_parent=\"%.3g\" P_total=\"%.3g\" P_dyn=\"%.3g\" transistors=\"%g\">\n",
			type->name, pb_type_power, pb_type_power / parent_power * 100,
			pb_type_power / total_power * 100,
			type->power_usage.dynamic / pb_type_power, type->transistor_cnt);

	mode_indent = 0;
	if (type->num_modes > 1) {
		mode_indent = 1;
	}

	for (mode_idx = 0; mode_idx < type->num_modes; mode_idx++) {
		float mode_power;
		mode_power = type->modes[mode_idx].power_usage.dynamic
				+ type->modes[mode_idx].power_usage.leakage;

		if (type->num_modes > 1) {
			print_tabs(fp, indent_level + mode_indent);
			fprintf(fp,
					"<mode name=\"%s\" P=\"%.4g\" P_parent=\"%.3g\" P_total=\"%.3g\" P_dyn=\"%.3g\">\n",
					type->modes[mode_idx].name, mode_power,
					mode_power / pb_type_power * 100,
					mode_power / total_power * 100,
					type->modes[mode_idx].power_usage.dynamic / mode_power);
		}

		if (type->modes[mode_idx].num_interconnect) {
			/* Sum the interconnect power */
			t_power_usage interc_power_usage;
			float interc_total_power;

			power_zero_usage(&interc_power_usage);
			for (interc_idx = 0;
					interc_idx < type->modes[mode_idx].num_interconnect;
					interc_idx++) {
				power_add_usage(&interc_power_usage,
						&type->modes[mode_idx].interconnect[interc_idx].power_usage);
			}
			interc_total_power = interc_power_usage.dynamic
					+ interc_power_usage.leakage;

			/* All interconnect */
			print_tabs(fp, indent_level + mode_indent + 1);
			fprintf(fp,
					"<interconnect P=\"%.4g\" P_parent=\"%.3g\" P_total=\"%.3g\" P_dyn=\"%.3g\">\n",
					interc_total_power, interc_total_power / mode_power * 100,
					interc_total_power / total_power * 100,
					interc_power_usage.dynamic / interc_total_power);
			for (interc_idx = 0;
					interc_idx < type->modes[mode_idx].num_interconnect;
					interc_idx++) {
				float interc_power =
						type->modes[mode_idx].interconnect[interc_idx].power_usage.dynamic
								+ type->modes[mode_idx].interconnect[interc_idx].power_usage.leakage;
				/* Each interconnect */
				print_tabs(fp, indent_level + mode_indent + 2);
				fprintf(fp,
						"<%s name=\"%s\" P=\"%.4g\" P_parent=\"%.3g\" P_total=\"%.3g\" P_dyn=\"%.3g\"/>\n",
						interconnect_type_name(
								type->modes[mode_idx].interconnect[interc_idx].type),
						type->modes[mode_idx].interconnect[interc_idx].name,
						interc_power, interc_power / interc_total_power * 100,
						interc_power / total_power * 100,
						type->modes[mode_idx].interconnect[interc_idx].power_usage.dynamic
								/ interc_power);
			}
			print_tabs(fp, indent_level + mode_indent + 1);
			fprintf(fp, "</interconnect>\n");
		}

		for (child_idx = 0;
				child_idx < type->modes[mode_idx].num_pb_type_children;
				child_idx++) {
			power_print_pb_usage_recursive(fp,
					&type->modes[mode_idx].pb_type_children[child_idx],
					indent_level + mode_indent + 1,
					type->modes[mode_idx].power_usage.dynamic
							+ type->modes[mode_idx].power_usage.leakage,
					total_power);
		}

		if (type->num_modes > 1) {
			print_tabs(fp, indent_level + mode_indent);
			fprintf(fp, "</mode>\n");
		}
	}

	print_tabs(fp, indent_level);
	fprintf(fp, "</pb_type>\n");
}

static float power_usage_combined(t_power_usage * power_usage) {
	return power_usage->dynamic + power_usage->leakage;
}

void power_print_pb_usage(FILE * fp) {
	int type_idx;

	for (type_idx = 0; type_idx < num_types; type_idx++) {
		if (!type_descriptors[type_idx].pb_type) {
			continue;
		}
		power_print_pb_usage_recursive(fp, type_descriptors[type_idx].pb_type,
				0,
				power_usage_combined(
						&g_power_breakdown->elements[POWER_ELEMENT_TILES]),
				power_usage_combined(
						&g_power_breakdown->elements[POWER_ELEMENT_TILES]));
	}
}

char binary_not(char c) {
	if (c == '1') {
		return '0';
	} else {
		return '1';
	}
}

void power_print_spice_comparison(void) {
	float leakage;
	t_power_usage sub_power_usage;

	float inv_sizes[3] = { 1, 16, 64 };

	float buffer_sizes[3] = { 4, 16, 64 };

	int LUT_sizes[3] = { 2, 4, 6 };

	float sb_buffer_sizes[5] = { 9, 9, 16, 64, 64 };
	int sb_mux_sizes[5] = { 4, 7, 12, 16, 20 };

	unsigned int i, j;
	float * dens = NULL;
	float * prob = NULL;
	char * SRAM_bits = NULL;
	int sram_idx;

	leakage = power_calc_transistor_leakage(NMOS, 1);
	fprintf(g_power_output->out, "Leakage of min-size NMOS: %g\n", leakage);
	leakage = power_calc_transistor_leakage(PMOS,
			g_power_arch->P_to_N_size_ratio);
	fprintf(g_power_output->out, "Leakage of min-size PMOS: %g\n", leakage);

	fprintf(g_power_output->out, "Energy of INV\n");
	for (i = 0; i < (sizeof(inv_sizes) / sizeof(float)); i++) {
		power_calc_INV(&sub_power_usage, 2, 0.5,
				g_power_arch->P_to_N_size_ratio * inv_sizes[i], inv_sizes[i]);
		fprintf(g_power_output->out, "%g\t%g\n", inv_sizes[i],
				(sub_power_usage.dynamic + sub_power_usage.leakage)
						* g_solution_inf->T_crit);
	}

	fprintf(g_power_output->out, "Energy of Buffer\n");
	for (i = 0; i < (sizeof(buffer_sizes) / sizeof(float)); i++) {
		power_calc_buffer(&sub_power_usage, buffer_sizes[i], 2, 0.5);
		fprintf(g_power_output->out, "%g\t%g\n", buffer_sizes[i],
				(sub_power_usage.dynamic + sub_power_usage.leakage)
						* g_solution_inf->T_crit);
	}

	fprintf(g_power_output->out, "Energy of LUT\n");
	for (i = 0; i < (sizeof(LUT_sizes) / sizeof(int)); i++) {
		for (j = 1; j <= LUT_sizes[i]; j++) {
			SRAM_bits = my_realloc(SRAM_bits, ((1 << j) + 1) * sizeof(char));
			if (j == 1) {
				SRAM_bits[0] = '1';
				SRAM_bits[1] = '0';
			} else {
				for (sram_idx = 0; sram_idx < (1 << (j - 1)); sram_idx++) {
					SRAM_bits[sram_idx + (1 << (j - 1))] = binary_not(
							SRAM_bits[sram_idx]);
				}
			}
			SRAM_bits[1 << j] = '\0';
		}

		dens = my_realloc(dens, LUT_sizes[i] * sizeof(float));
		prob = my_realloc(prob, LUT_sizes[i] * sizeof(float));
		for (j = 0; j < LUT_sizes[i]; j++) {
			dens[j] = 1;
			prob[j] = 0.5;
		}
		power_calc_LUT(&sub_power_usage, LUT_sizes[i], SRAM_bits, dens, prob);
		fprintf(g_power_output->out, "%d\t%g\n", LUT_sizes[i],
				(sub_power_usage.dynamic + sub_power_usage.leakage)
						* g_solution_inf->T_crit * 2);
	}

	fprintf(g_power_output->out, "Energy of FF\n");
	power_calc_FF(&sub_power_usage, 3, 0.5, 1, 0.5, 2, 0.5);
	fprintf(g_power_output->out, "%g\n",
			(sub_power_usage.dynamic + sub_power_usage.leakage)
					* g_solution_inf->T_crit * 2);

	fprintf(g_power_output->out, "Energy of Mux\n");
	for (i = 1; i <= 0; i++) {
		t_power_usage mux_power_usage;

		power_zero_usage(&mux_power_usage);

		dens = my_realloc(dens, i * sizeof(float));
		prob = my_realloc(prob, i * sizeof(float));
		dens[i - 1] = 2;
		prob[i - 1] = 0.5;

		power_calc_MUX(&mux_power_usage, &g_power_common->mux_arch[i], dens,
				prob, 0);
		fprintf(g_power_output->out, "%d\t%g\n", i,
				(mux_power_usage.dynamic + mux_power_usage.leakage)
						* g_solution_inf->T_crit);
	}

	fprintf(g_power_output->out, "Energy of SB\n");
	for (i = 0; i < (sizeof(sb_buffer_sizes) / sizeof(float)); i++) {
		t_power_usage sb_power_usage;

		power_zero_usage(&sb_power_usage);

		dens = my_realloc(dens, sb_mux_sizes[i] * sizeof(float));
		prob = my_realloc(prob, sb_mux_sizes[i] * sizeof(float));
		for (j = 0; j < sb_mux_sizes[i]; j++) {
			dens[j] = 2;
			prob[j] = 0.5;
		}

		power_calc_MUX(&sub_power_usage,
				&g_power_common->mux_arch[sb_mux_sizes[i]], dens, prob, 0);
		power_add_usage(&sb_power_usage, &sub_power_usage);

		power_calc_buffer(&sub_power_usage, sb_buffer_sizes[i], 2, 0.5);
		power_add_usage(&sb_power_usage, &sub_power_usage);

		fprintf(g_power_output->out, "%d\t%.0f\t%g\n", sb_mux_sizes[i],
				sb_buffer_sizes[i],
				(sb_power_usage.dynamic + sb_power_usage.leakage)
						* g_solution_inf->T_crit);
	}

}

void power_print_title(FILE * fp, char * title) {
	int i;
	const int width = 80;

	int firsthalf = (width - strlen(title) - 2) / 2;
	int secondhalf = width - strlen(title) - 2 - firsthalf;

	for (i = 1; i < firsthalf; i++)
		fprintf(fp, "-");
	fprintf(fp, " %s ", title);
	for (i = 1; i < secondhalf; i++)
		fprintf(fp, "-");
	fprintf(fp, "\n");
}

void power_print_stats(FILE * fp) {
	fprintf(fp, "Technology (nm): %.0f\n",
			g_power_arch->tech_size * CONVERT_NM_PER_M);
	fprintf(fp, "Voltage: %.2f\n", g_power_arch->Vdd);
	fprintf(fp, "Operating temperature: %g\n", g_power_arch->temperature);

	fprintf(fp, "Layout of FPGA: %d x %d\n", nx, ny);

	fprintf(fp, "Max Segment Fanout: %d\n", g_power_common->max_seg_fanout);
	fprintf(fp, "Max Segment->Segment Fanout: %d\n",
			g_power_common->max_seg_to_seg_fanout);
	fprintf(fp, "Max Segment->IPIN Fanout: %d\n",
			g_power_common->max_seg_to_IPIN_fanout);
	fprintf(fp, "Max IPIN fanin: %d\n", g_power_common->max_IPIN_fanin);
	fprintf(fp, "Average SB Buffer Size: %.1f\n",
			g_power_common->total_sb_buffer_size
					/ (float) g_power_common->num_sb_buffers);
	fprintf(fp, "SB Buffer Transistors: %g\n",
			power_count_transistors_buffer(
					g_power_common->total_sb_buffer_size
							/ (float) g_power_common->num_sb_buffers));
	fprintf(fp, "Average CB Buffer Size: %.1f\n",
			g_power_common->total_cb_buffer_size
					/ (float) g_power_common->num_cb_buffers);
	fprintf(fp, "Tile length (um): %.2f\n",
			g_power_common->tile_length * CONVERT_UM_PER_M);
	fprintf(fp, "1X Inverter C_in: %g\n", g_power_common->INV_1X_C_in);
	fprintf(fp, "\n");

}

/*
 * Func Name	: 	calc_power
 *
 * Description	: 	Calculated the power required during a clock cycle of the entire circuit.
 *
 * Arguments	: 	power_options - Power Options
 * 					power_architecture - Power Architecture
 * 					directionality - Bi/Uni Directional
 * 					switch_block_type - Switch block type used in the architecture
 * 					T_crit - Period of the critical path
 * 					num_segment_types - Number of different types of routing segments avail. in the architecture
 * 					segment_inf - Information for each routing segment - Array 0..(num_segment-1)
 *					total_segments_used - Total number of segments used in the routed design
 *					num_switch_types - Number of types of switches in the architecture (includes 2 switches added by VPR for internal purposes)
 *
 * Returns		:	Power in Watts
 */
e_power_ret_code power_total(void) {
	t_power_usage total_power;
	t_power_usage sub_power_usage;
	int i;
	t_power_usage * cb_power_usage;
	int type_idx;

	total_power.dynamic = 0.;
	total_power.leakage = 0.;

	if (g_routing_arch->directionality == BI_DIRECTIONAL) {
		power_log_msg(POWER_LOG_ERROR,
				"Cannot calculate routing power for bi-directional architectures");
		return POWER_RET_CODE_ERRORS;
	}

	if (PRINT_SPICE_COMPARISON) {
		power_print_spice_comparison();
	}

	/* Calculate Power */

	/* Routing */
	power_calc_routing(&sub_power_usage);
	power_add_usage_element(&total_power, &sub_power_usage,
			POWER_ELEMENT_ROUTING);

	/* Clock  */
	power_calc_clock(&sub_power_usage, g_arch->clocks);
	power_add_usage_element(&total_power, &sub_power_usage,
			POWER_ELEMENT_CLOCK);

	/* Complex Blocks */
	cb_power_usage = malloc(num_types * sizeof(t_power_usage));
	for (type_idx = 0; type_idx < num_types; type_idx++) {
		cb_power_usage[type_idx].dynamic = 0.;
		cb_power_usage[type_idx].leakage = 0.;
	}
	power_calc_tile_usage(cb_power_usage);

	for (type_idx = 0; type_idx < num_types; type_idx++) {
		power_add_usage_element(&total_power, &cb_power_usage[type_idx],
				POWER_ELEMENT_TILES);
	}

	power_add_usage_element(NULL, &total_power, POWER_ELEMENT_TOTAL);

	/* Print Error & Warning Logs */
	output_logs(g_power_output->out, g_power_output->logs,
			g_power_output->num_logs);

	power_print_title(g_power_output->out, "Statistics");
	power_print_stats(g_power_output->out);

	power_print_title(g_power_output->out, "Power By Element");

	power_print_element_usage("Total", POWER_ELEMENT_TOTAL, POWER_ELEMENT_TOTAL,
			0);

	power_print_title(g_power_output->out, "Tile Power Breakdown");
	power_print_pb_usage(g_power_output->out);

	free(cb_power_usage);

	/* Return code */
	if (g_power_output->logs[POWER_LOG_ERROR].num_messages) {
		return POWER_RET_CODE_ERRORS;
	} else if (g_power_output->logs[POWER_LOG_WARNING].num_messages) {
		return POWER_RET_CODE_WARNINGS;
	} else {
		return POWER_RET_CODE_SUCCESS;
	}
}

void log_msg(t_log * log_ptr, char * msg) {
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

void power_log_msg(enum e_power_log_type log_type, char * msg) {
	log_msg(&g_power_output->logs[log_type], msg);
}

void output_log(t_log * log_ptr, FILE * fp) {
	int msg_idx;

	for (msg_idx = 0; msg_idx < log_ptr->num_messages; msg_idx++) {
		fprintf(fp, "%s\n", log_ptr->messages[msg_idx]);
	}
}

void output_logs(FILE * fp, t_log * logs, int num_logs) {
	int log_idx;

	for (log_idx = 0; log_idx < num_logs; log_idx++) {
		if (logs[log_idx].num_messages) {
			power_print_title(fp, logs[log_idx].name);
			output_log(&logs[log_idx], fp);
			fprintf(fp, "\n");
		}
	}
}

void power_calc_INV(t_power_usage * power_usage, float in_density,
		float in_probability, float PMOS_size, float NMOS_size) {
	float C_drain, C_gate, C_source;
	float C_inv;

	power_usage->dynamic = 0.;
	power_usage->leakage = 0.;

	C_inv = 0.;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS,
			NMOS_size, POWER_CIRCUIT_TYPE_CMOS);
	C_inv += C_gate + C_drain;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, PMOS,
			PMOS_size, POWER_CIRCUIT_TYPE_CMOS);
	C_inv += C_gate + C_drain;

	power_usage->dynamic = power_calc_dynamic(C_inv, in_density);
	power_usage->leakage = in_probability
			* power_calc_transistor_leakage(PMOS, PMOS_size)
			+ (1 - in_probability)
					* power_calc_transistor_leakage(NMOS, NMOS_size);
}

void power_calc_INV1(t_power_usage * power_usage, float in_density,
		float in_probability) {
	power_usage->dynamic = power_calc_dynamic(g_power_common->INV_1X_C,
			in_density);
	power_usage->leakage = (1 - in_probability)
			* g_power_common->NMOS_1X_leakage
			+ in_probability * g_power_common->PMOS_1X_leakage;
}

void power_calc_INV2(t_power_usage * power_usage, float in_density,
		float in_probability) {
	power_usage->dynamic = power_calc_dynamic(g_power_common->INV_2X_C,
			in_density);
	power_usage->leakage = (1 - in_probability)
			* g_power_common->NMOS_2X_leakage
			+ in_probability * g_power_common->PMOS_2X_leakage;
}

void power_calc_MUX_node(t_power_usage * power_usage, float * out_dens,
		float * out_prob, t_mux_node * mux_node, t_mux_arch * mux_arch,
		int * selector_values, float * primary_input_density,
		float * primary_input_prob) {
	int input_idx;
	float C_drain, C_source, C_gate;
	float leakage;
	float * in_prob;
	float output_prob;
	float Vin;

	/* Single input mux is really just a wire, and has no power.
	 * Ensure that it has no children before returning. */
	if (mux_node->num_inputs == 1) {
		assert(mux_node->level == 0);
		return;
	}

	(*out_dens) = 0;
	output_prob = 0.;

	in_prob = my_calloc(mux_node->num_inputs, sizeof(float));

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS,
			mux_arch->transistor_sizes[mux_node->level],
			POWER_CIRCUIT_TYPE_PASS);

	/* Leakage of all transistors */
	leakage = power_calc_transistor_leakage(NMOS,
			mux_arch->transistor_sizes[mux_node->level]);

	/* Input Voltage */
	if (mux_node->level == 0) {
		Vin = g_power_arch->Vdd;
	} else {
		Vin = g_power_arch->Vdd - g_power_arch->NMOS_tx_record.Vth;
	}

	for (input_idx = 0; input_idx < mux_node->num_inputs; input_idx++) {
		float in_density;

		if (mux_node->level == 0) {
			in_density = primary_input_density[mux_node->starting_pin_idx
					+ input_idx];
			in_prob[input_idx] = primary_input_prob[mux_node->starting_pin_idx
					+ input_idx];
		} else {
			power_calc_MUX_node(power_usage, &in_density, &in_prob[input_idx],
					&mux_node->children[input_idx], mux_arch, selector_values,
					primary_input_density, primary_input_prob);
		}

		if (input_idx == selector_values[mux_node->level]) {
			(*out_dens) = in_density;
			output_prob = in_prob[input_idx];
		}

		/* Dynamic Power at Inputs */
		power_usage->dynamic += power_calc_dynamic_v(C_drain, in_density, Vin);
	}

	/* Calculate leakage for all input transistors */
	for (input_idx = 0; input_idx < mux_node->num_inputs; input_idx++) {

		/* The selected input will never leak */
		if (input_idx == selector_values[mux_node->level]) {
			continue;
		}

		/* Leakage occurs when output != input */
		power_usage->leakage += ((1 - in_prob[input_idx]) * output_prob
				+ in_prob[input_idx] * (1 - output_prob)) * leakage;
	}

	/* Dynamic Power at Output */
	power_usage->dynamic += power_calc_dynamic_v(
			C_source * mux_node->num_inputs, (*out_dens),
			g_power_arch->Vdd - g_power_arch->NMOS_tx_record.Vth);

	*out_prob = output_prob;
}

static void power_calc_level_restorer(t_power_usage * power_usage,
		float in_density, float in_probability) {
	t_power_usage sub_power_usage;
	float C;

	power_zero_usage(power_usage);

	/* Inverter */
	power_calc_INV(&sub_power_usage, in_density, in_probability, 1.0, 2.0);
	power_add_usage(power_usage, &sub_power_usage);

	if (g_power_arch->PMOS_tx_record.long_trans_inf == NULL) {
		power_log_msg(POWER_LOG_ERROR,
				"No long transistor information exists.  Cannot determine transistor properties.");
		return;
	}
	C = g_power_arch->PMOS_tx_record.long_trans_inf->C_drain_cmos
			+ g_power_arch->PMOS_tx_record.long_trans_inf->C_gate_cmos;

	power_usage->dynamic += power_calc_dynamic(C, in_density);
	power_usage->leakage += (1 - in_probability)
			* g_power_arch->PMOS_tx_record.long_trans_inf->leakage;

}

static void power_calc_FF(t_power_usage * power_usage, float D_dens,
		float D_prob, float Q_dens, float Q_prob, float clk_dens,
		float clk_prob) {
	t_power_usage sub_power_usage;
	float mux_in_dens[2];
	float mux_in_prob[2];

	power_zero_usage(power_usage);

	/* DFF is build using a master loop and slave loop.
	 * Each loop begins with a MUX and contains 2 inverters
	 * in a feedback loop to the mux.
	 * Each mux is built using two transmission gates.
	 */

	/* Master */
	mux_in_dens[0] = D_dens;
	mux_in_dens[1] = (1 - clk_prob) * D_dens;
	mux_in_prob[0] = D_prob;
	mux_in_prob[1] = D_prob;
	power_calc_MUX2_transmission(&sub_power_usage, mux_in_dens, mux_in_prob,
			clk_dens, clk_prob, (1 - clk_prob) * D_dens);
	power_add_usage(power_usage, &sub_power_usage);

	power_calc_INV1(&sub_power_usage, (1 - clk_prob) * D_dens, D_prob);
	power_add_usage(power_usage, &sub_power_usage);

	power_calc_INV1(&sub_power_usage, (1 - clk_prob) * D_dens, 1 - D_prob);
	power_add_usage(power_usage, &sub_power_usage);

	/* Slave */
	mux_in_dens[0] = Q_dens;
	mux_in_dens[1] = (1 - clk_prob) * D_dens;
	mux_in_prob[0] = (1 - Q_prob);
	mux_in_prob[1] = (1 - D_prob);
	power_calc_MUX2_transmission(&sub_power_usage, mux_in_dens, mux_in_prob,
			clk_dens, clk_prob, Q_dens);
	power_add_usage(power_usage, &sub_power_usage);

	power_calc_INV1(&sub_power_usage, Q_dens, 1 - Q_prob);
	power_add_usage(power_usage, &sub_power_usage);

	power_calc_INV1(&sub_power_usage, Q_dens, Q_prob);
	power_add_usage(power_usage, &sub_power_usage);

	return;
}

static void power_calc_MUX2_transmission(t_power_usage * power_usage,
		float * in_dens, float * in_prob, float sel_dens, float sel_prob,
		float out_dens) {

	power_zero_usage(power_usage);

	/* A transmission gate leaks if the selected input != other input  */
	power_usage->leakage +=
			(in_prob[0] * (1 - in_prob[1]) + (1 - in_prob[0]) * in_prob[1])
					* (g_power_common->NMOS_1X_leakage
							+ g_power_common->PMOS_1X_leakage);

	/* Gate switching */
	power_usage->dynamic += 2
			* power_calc_dynamic(
					g_power_common->NMOS_1X_C_gate_avg
							+ g_power_common->PMOS_1X_C_gate_avg, sel_dens);

	/* Input switching */
	power_usage->dynamic += power_calc_dynamic(
			g_power_common->NMOS_1X_C_drain_avg
					+ g_power_common->PMOS_1X_C_source_avg, in_dens[0]);
	power_usage->dynamic += power_calc_dynamic(
			g_power_common->NMOS_1X_C_drain_avg
					+ g_power_common->PMOS_1X_C_source_avg, in_dens[1]);

	/* Output switching */
	power_usage->dynamic += power_calc_dynamic(
			2
					* (g_power_common->NMOS_1X_C_source_avg
							+ g_power_common->PMOS_1X_C_drain_avg), out_dens);
}

static void power_calc_MUX2(t_power_usage * power_usage, float Vin, float Vout,
		float out_density, float * in_density, float * in_prob, float sel_dens,
		float sel_prob) {

	power_zero_usage(power_usage);

	/* A transistor leaks if the selected input != other input */
	power_usage->leakage += (in_prob[0] * (1 - in_prob[1])
			+ (1 - in_prob[0]) * in_prob[1]) * g_power_common->NMOS_1X_leakage;

	/* Gate switching */
	power_usage->dynamic += 2
			* power_calc_dynamic(g_power_common->NMOS_1X_C_gate_avg, sel_dens);

	/* Input switching */
	power_usage->dynamic += power_calc_dynamic_v(
			g_power_common->NMOS_1X_C_drain_avg, in_density[0], Vin);
	power_usage->dynamic += power_calc_dynamic_v(
			g_power_common->NMOS_1X_C_drain_avg, in_density[1], Vin);

	/* Output switching */
	power_usage->dynamic += power_calc_dynamic_v(
			2 * g_power_common->NMOS_1X_C_source_avg, out_density, Vout);
}

/*
 * Func Name	:	calc_transistor_capacitance
 *
 * Description	: 	Calculate the capacitance for a transistor
 *
 * Arguments	: 	C_drain - Return value of drain capacitance
 * 					C_source - Return value of source capacitance
 * 					C_gate - Return value of gate capacitacne
 * 					transistor_type - NMOS or PMOS
 * 					size - Transistor size (relative to min-size)
 *
 * Returns		: 	Nothing
 */
void power_calc_transistor_capacitance(float *C_drain, float *C_source,
		float *C_gate, e_tx_type transistor_type, float size,
		e_power_circuit_type circuit_type) {
	t_transistor_size_inf * tx_info_lower;
	t_transistor_size_inf * tx_info_upper;
	boolean error;

	assert(
			circuit_type == POWER_CIRCUIT_TYPE_CMOS || circuit_type == POWER_CIRCUIT_TYPE_PASS);

	/* Initialize to 0 */
	*C_drain = 0.;
	*C_source = 0.;
	*C_gate = 0.;

	error = power_find_transistor_info(&tx_info_lower, &tx_info_upper,
			transistor_type, size);
	if (error) {
		return;
	}

	if (tx_info_lower == NULL) {
		/* No lower bound */
		if (circuit_type == POWER_CIRCUIT_TYPE_CMOS) {
			*C_drain = tx_info_upper->C_drain_cmos;
			*C_source = tx_info_upper->C_source_cmos;
			*C_gate = tx_info_upper->C_gate_cmos;
		} else if (circuit_type == POWER_CIRCUIT_TYPE_PASS) {
			*C_drain = tx_info_upper->C_drain_pass;
			*C_source = tx_info_upper->C_source_pass;
			*C_gate = tx_info_upper->C_gate_pass;
		}
	} else if (tx_info_upper == NULL) {
		/* No upper bound */
		if (circuit_type == POWER_CIRCUIT_TYPE_CMOS) {
			*C_drain = tx_info_lower->C_drain_cmos;
			*C_source = tx_info_lower->C_source_cmos;
			*C_gate = tx_info_lower->C_gate_cmos;
		} else if (circuit_type == POWER_CIRCUIT_TYPE_PASS) {
			*C_drain = tx_info_lower->C_drain_pass;
			*C_source = tx_info_lower->C_source_pass;
			*C_gate = tx_info_lower->C_gate_pass;
		}
	} else {
		/* Linear approximation between sizes */
		float percent_upper = (size - tx_info_lower->size)
				/ (tx_info_upper->size - tx_info_lower->size);
		if (circuit_type == POWER_CIRCUIT_TYPE_CMOS) {
			*C_drain = (1 - percent_upper) * tx_info_lower->C_drain_cmos
					+ percent_upper * tx_info_upper->C_drain_cmos;
			*C_source = (1 - percent_upper) * tx_info_lower->C_source_cmos
					+ percent_upper * tx_info_upper->C_source_cmos;
			*C_gate = (1 - percent_upper) * tx_info_lower->C_gate_cmos
					+ percent_upper * tx_info_upper->C_gate_cmos;
		} else if (circuit_type == POWER_CIRCUIT_TYPE_PASS) {
			*C_drain = (1 - percent_upper) * tx_info_lower->C_drain_pass
					+ percent_upper * tx_info_upper->C_drain_pass;
			*C_source = (1 - percent_upper) * tx_info_lower->C_source_pass
					+ percent_upper * tx_info_upper->C_source_pass;
			*C_gate = (1 - percent_upper) * tx_info_lower->C_gate_pass
					+ percent_upper * tx_info_upper->C_gate_pass;
		}
	}

	return;
}

float power_calc_dynamic(float capacitance, float density) {
	return 0.5 * g_power_arch->Vdd * g_power_arch->Vdd
			* (1 + g_power_arch->short_circuit_power_percentage) * capacitance
			* density / g_solution_inf->T_crit;
}

float power_calc_dynamic_v(float capacitance, float density, float voltage) {
	return 0.5 * voltage * g_power_arch->Vdd
			* (1 + g_power_arch->short_circuit_power_percentage) * capacitance
			* density / g_solution_inf->T_crit;
}

/*
 * Func Name	: 	power_calc_transistor_leakage
 *
 * Description	: 	Calculates the leakage power (Watts) for a transistor
 *
 * Arguments	:	transistor_type - NMOS or PMOS
 * 					C_diffusion - Diffusion capacitance
 * 					size - Size of transistor, relative to minimum size
 *
 * Returns		: 	Leakage power (Watts)
 */
float power_calc_transistor_leakage(e_tx_type transistor_type, float size) {
	t_transistor_size_inf * tx_info_lower;
	t_transistor_size_inf * tx_info_upper;
	boolean error;
	float current;

	error = power_find_transistor_info(&tx_info_lower, &tx_info_upper,
			transistor_type, size);
	if (error) {
		return 0;
	}

	if (tx_info_lower == NULL) {
		/* No lower bound */
		current = tx_info_upper->leakage;
	} else if (tx_info_upper == NULL) {
		/* No upper bound */
		current = tx_info_lower->leakage;
	} else {
		/* Linear approximation between sizes */
		float percent_upper = (size - tx_info_lower->size)
				/ (tx_info_upper->size - tx_info_lower->size);
		current = (1 - percent_upper) * tx_info_lower->leakage
				+ percent_upper * tx_info_upper->leakage;
	}

	return current * g_power_arch->Vdd;
}

