/************************* INCLUDES *********************************/
#include <assert.h>

#include "power_lowlevel.h"
#include "power_util.h"
#include "power_cmos_tech.h"
#include "globals.h"

/************************* FUNCTION DELCARATIONS ********************/
static float power_calc_dynamic(float capacitance, float density);
static float power_calc_dynamic_v(float capacitance, float density,
		float voltage);
static void power_calc_transistor_capacitance(float *C_drain, float *C_source,
		float *C_gate, e_tx_type transistor_type, float size,
		e_power_circuit_type circuit_type);
static float power_calc_leakage_subthreshold(e_tx_type transistor_type,
		float size);
static float power_calc_leakage_pass_transistor(float size, float v_ds);
static float power_calc_leakage_gate(e_tx_type transistor_type, float size);

/************************* FUNCTION DEFINITIONS *********************/
void power_lowlevel_init() {
	float C_drain, C_source, C_gate;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS, 1.0,
			POWER_CIRCUIT_TYPE_CMOS);
	g_power_commonly_used->NMOS_1X_C_drain_cmos = C_drain;
	g_power_commonly_used->NMOS_1X_C_gate_cmos = C_gate;
	g_power_commonly_used->NMOS_1X_C_source_cmos = C_source;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS, 1.0,
			POWER_CIRCUIT_TYPE_PASS);
	g_power_commonly_used->NMOS_1X_C_drain_pass = C_drain;
	g_power_commonly_used->NMOS_1X_C_gate_pass = C_gate;
	g_power_commonly_used->NMOS_1X_C_source_pass = C_source;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS,
			g_power_tech->PN_ratio, POWER_CIRCUIT_TYPE_CMOS);
	g_power_commonly_used->PMOS_1X_C_drain_cmos = C_drain;
	g_power_commonly_used->PMOS_1X_C_gate_cmos = C_gate;
	g_power_commonly_used->PMOS_1X_C_source_cmos = C_source;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS,
			g_power_tech->PN_ratio, POWER_CIRCUIT_TYPE_PASS);
	g_power_commonly_used->PMOS_1X_C_drain_pass = C_drain;
	g_power_commonly_used->PMOS_1X_C_gate_pass = C_gate;
	g_power_commonly_used->PMOS_1X_C_source_pass = C_source;

	g_power_commonly_used->NMOS_1X_leakage_cmos =
			power_calc_leakage_subthreshold(NMOS, 1.0);
	g_power_commonly_used->PMOS_1X_leakage_cmos =
			power_calc_leakage_subthreshold(PMOS, 1.0 * g_power_tech->PN_ratio);

	g_power_commonly_used->NMOS_2X_leakage_cmos =
			power_calc_leakage_subthreshold(NMOS, 2.0);
	g_power_commonly_used->PMOS_2X_leakage_cmos =
			power_calc_leakage_subthreshold(PMOS, 2.0 * g_power_tech->PN_ratio);

	g_power_commonly_used->INV_1X_C_in =
			g_power_commonly_used->NMOS_1X_C_gate_cmos
					+ g_power_commonly_used->PMOS_1X_C_gate_cmos;
	g_power_commonly_used->INV_1X_C = g_power_commonly_used->NMOS_1X_C_gate_cmos
			+ g_power_commonly_used->PMOS_1X_C_gate_cmos
			+ g_power_commonly_used->NMOS_1X_C_drain_cmos
			+ g_power_commonly_used->PMOS_1X_C_drain_cmos;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS, 2.0,
			POWER_CIRCUIT_TYPE_CMOS);
	g_power_commonly_used->INV_2X_C = C_gate + C_drain;
	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, PMOS,
			2.0 * g_power_tech->PN_ratio, POWER_CIRCUIT_TYPE_CMOS);
	g_power_commonly_used->INV_2X_C = C_gate + C_drain;

}

static float power_calc_dynamic(float capacitance, float density) {
	return 0.5 * g_power_tech->Vdd * g_power_tech->Vdd * capacitance * density
			/ g_solution_inf->T_crit;
}

static float power_calc_dynamic_v(float capacitance, float density,
		float voltage) {
	return 0.5 * voltage * g_power_tech->Vdd * capacitance * density
			/ g_solution_inf->T_crit;
}

void power_calc_inverter_irregular(t_power_usage * power_usage, float * dyn_power_input,
		float in_density, float in_probability, float PMOS_size,
		float NMOS_size) {
	float C_drain, C_gate, C_source;
	float C_inv;
	float C_in;

	power_usage->dynamic = 0.;
	power_usage->leakage = 0.;

	C_inv = 0.;
	C_in = 0.;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS,
			NMOS_size, POWER_CIRCUIT_TYPE_CMOS);
	C_inv += C_gate + C_drain;
	C_in += C_gate;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, PMOS,
			PMOS_size, POWER_CIRCUIT_TYPE_CMOS);
	C_inv += C_gate + C_drain;
	C_in += C_gate;

	power_usage->dynamic = power_calc_dynamic(C_inv, in_density);
	*dyn_power_input = power_calc_dynamic(C_in, in_density);

	power_usage->leakage = in_probability
			* power_calc_leakage_subthreshold(PMOS, PMOS_size)
			+ (1 - in_probability)
					* power_calc_leakage_subthreshold(NMOS, NMOS_size);
}

void power_calc_inverter_with_input(t_power_usage * power_usage, float * input_dynamic_power, float in_density,
		float in_prob, float size) {
	float C_drain, C_gate, C_source;
	float C_inv;
	float C_in;

	float PMOS_size = g_power_tech->PN_ratio * size;
	float NMOS_size = size;

	power_usage->dynamic = 0.;
	power_usage->leakage = 0.;

	C_inv = 0.;
	C_in = 0.;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS,
			NMOS_size, POWER_CIRCUIT_TYPE_CMOS);
	C_inv += C_gate + C_drain;
	C_in += C_gate;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, PMOS,
			PMOS_size, POWER_CIRCUIT_TYPE_CMOS);
	C_inv += C_gate + C_drain;
	C_in += C_gate;

	power_usage->dynamic = power_calc_dynamic(C_inv, in_density);
	*input_dynamic_power = power_calc_dynamic(C_in, in_density);

	power_usage->leakage = in_prob
			* power_calc_leakage_subthreshold(PMOS, PMOS_size)
			+ (1 - in_prob) * power_calc_leakage_subthreshold(NMOS, NMOS_size);

	power_usage->leakage += in_prob * power_calc_leakage_gate(NMOS, NMOS_size)
			+ (1 - in_prob) * power_calc_leakage_gate(PMOS, PMOS_size);
}

void power_calc_inverter(t_power_usage * power_usage, float in_density,
		float in_prob, float size) {
	float C_drain, C_gate, C_source;
	float C_inv;

	float PMOS_size = g_power_tech->PN_ratio * size;
	float NMOS_size = size;

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

	power_usage->leakage = in_prob
			* power_calc_leakage_subthreshold(PMOS, PMOS_size)
			+ (1 - in_prob) * power_calc_leakage_subthreshold(NMOS, NMOS_size);

	power_usage->leakage += in_prob * power_calc_leakage_gate(NMOS, NMOS_size)
			+ (1 - in_prob) * power_calc_leakage_gate(PMOS, PMOS_size);
}

void power_calc_inverter_nogate(t_power_usage * power_usage, float in_density,
		float in_probability, float PMOS_size, float NMOS_size) {
	float C_drain, C_gate, C_source;
	float C_inv;

	power_usage->dynamic = 0.;
	power_usage->leakage = 0.;

	C_inv = 0.;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS,
			NMOS_size, POWER_CIRCUIT_TYPE_CMOS);
	C_inv += C_drain;

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, PMOS,
			PMOS_size, POWER_CIRCUIT_TYPE_CMOS);
	C_inv += C_drain;

	power_usage->dynamic = power_calc_dynamic(C_inv, in_density);
	power_usage->leakage = in_probability
			* power_calc_leakage_subthreshold(PMOS, PMOS_size)
			+ (1 - in_probability)
					* power_calc_leakage_subthreshold(NMOS, NMOS_size);
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
static void power_calc_transistor_capacitance(float *C_drain, float *C_source,
		float *C_gate, e_tx_type transistor_type, float size,
		e_power_circuit_type circuit_type) {
	t_transistor_size_inf * tx_info_lower;
	t_transistor_size_inf * tx_info_upper;
	boolean error;

	circuit_type = POWER_CIRCUIT_TYPE_CMOS;

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

	if (tx_info_lower == NULL ) {
		/* No lower bound */
		if (circuit_type == POWER_CIRCUIT_TYPE_CMOS) {
			*C_drain = tx_info_upper->C_drain_cmos;
			*C_source = tx_info_upper->C_drain_cmos;
			*C_gate = tx_info_upper->C_gate_cmos;
		} else if (circuit_type == POWER_CIRCUIT_TYPE_PASS) {
			*C_drain = tx_info_upper->C_drain_pass;
			*C_source = tx_info_upper->C_source_pass;
			*C_gate = tx_info_upper->C_gate_pass;
		}
	} else if (tx_info_upper == NULL ) {
		/* No upper bound */
		if (circuit_type == POWER_CIRCUIT_TYPE_CMOS) {
			*C_drain = tx_info_lower->C_drain_cmos;
			*C_source = tx_info_lower->C_drain_cmos;
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
			*C_source = (1 - percent_upper) * tx_info_lower->C_drain_cmos
					+ percent_upper * tx_info_upper->C_drain_cmos;
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
static float power_calc_leakage_subthreshold(e_tx_type transistor_type,
		float size) {
	t_transistor_size_inf * tx_info_lower;
	t_transistor_size_inf * tx_info_upper;
	boolean error;
	float current;

	error = power_find_transistor_info(&tx_info_lower, &tx_info_upper,
			transistor_type, size);
	if (error) {
		return 0;
	}

	if (tx_info_lower == NULL ) {
		/* No lower bound */
		current = tx_info_upper->leakage_subthreshold;

	} else if (tx_info_upper == NULL ) {
		/* No upper bound */
		current = tx_info_lower->leakage_subthreshold;
	} else {
		/* Linear approximation between sizes */
		float percent_upper = (size - tx_info_lower->size)
				/ (tx_info_upper->size - tx_info_lower->size);
		current = (1 - percent_upper) * tx_info_lower->leakage_subthreshold
				+ percent_upper * tx_info_upper->leakage_subthreshold;
	}

	return current * g_power_tech->Vdd;
}

static float power_calc_leakage_gate(e_tx_type transistor_type, float size) {
	t_transistor_size_inf * tx_info_lower;
	t_transistor_size_inf * tx_info_upper;
	boolean error;
	float current;

	error = power_find_transistor_info(&tx_info_lower, &tx_info_upper,
			transistor_type, size);
	if (error) {
		return 0;
	}

	if (tx_info_lower == NULL ) {
		/* No lower bound */
		current = tx_info_upper->leakage_gate;

	} else if (tx_info_upper == NULL ) {
		/* No upper bound */
		current = tx_info_lower->leakage_gate;
	} else {
		/* Linear approximation between sizes */
		float percent_upper = (size - tx_info_lower->size)
				/ (tx_info_upper->size - tx_info_lower->size);
		current = (1 - percent_upper) * tx_info_lower->leakage_gate
				+ percent_upper * tx_info_upper->leakage_gate;
	}

	return current * g_power_tech->Vdd;
}

static float power_calc_leakage_pass_transistor(float size, float v_ds) {
	t_power_nmos_leakage_pair * lower;
	t_power_nmos_leakage_pair * upper;
	float i_ds;

	assert(size == 1.0);

	power_find_nmos_leakage(&lower, &upper, v_ds);

	if (lower->v_ds == v_ds) {
		i_ds = lower->i_ds;
	} else {
		float perc_upper = (v_ds - lower->v_ds) / (upper->v_ds - lower->v_ds);
		i_ds = (1 - perc_upper) * lower->i_ds + perc_upper * upper->i_ds;
	}

	return i_ds * g_power_tech->Vdd;
}

void power_calc_wire(t_power_usage * power_usage, float capacitance,
		float density) {
	power_usage->leakage = 0.;
	power_usage->dynamic = power_calc_dynamic(capacitance, density);
}

void power_calc_MUX2_transmission(t_power_usage * power_usage, float * in_dens,
		float * in_prob, float sel_dens, float sel_prob, float out_dens) {

	power_zero_usage(power_usage);

	/* A transmission gate leaks if the selected input != other input  */
	power_usage->leakage += (in_prob[0] * (1 - in_prob[1])
			+ (1 - in_prob[0]) * in_prob[1])
			* (g_power_commonly_used->NMOS_1X_leakage_cmos
					+ g_power_commonly_used->PMOS_1X_leakage_cmos);

	/* Gate switching */
	power_usage->dynamic += 2
			* power_calc_dynamic(
					g_power_commonly_used->NMOS_1X_C_gate_pass
							+ g_power_commonly_used->PMOS_1X_C_gate_pass,
					sel_dens);

	/* Input switching */
	power_usage->dynamic += power_calc_dynamic(
			g_power_commonly_used->NMOS_1X_C_drain_pass
					+ g_power_commonly_used->PMOS_1X_C_source_pass, in_dens[0]);
	power_usage->dynamic += power_calc_dynamic(
			g_power_commonly_used->NMOS_1X_C_drain_pass
					+ g_power_commonly_used->PMOS_1X_C_source_pass, in_dens[1]);

	/* Output switching */
	power_usage->dynamic += power_calc_dynamic(
			2
					* (g_power_commonly_used->NMOS_1X_C_source_pass
							+ g_power_commonly_used->PMOS_1X_C_drain_pass),
			out_dens);
}

void power_calc_mux_singlelevel_static(t_power_usage * power_usage,
		float * out_prob, float * out_dens, float * v_out, int num_inputs,
		int selected_idx, float * in_prob, float * in_dens, float * v_in,
		float transistor_size, boolean v_out_restored) {
	int input_idx;
	float C_drain, C_source, C_gate;
	float v_in_selected;
	float in_prob_avg;

	power_zero_usage(power_usage);

	assert(transistor_size == 1.0);

	if (selected_idx < num_inputs) {
		*out_prob = in_prob[selected_idx];
		*out_dens = in_dens[selected_idx];
		v_in_selected = v_in[selected_idx];

	} else {
		/* In this case, the multiplexer is not symetrical.  The
		 * other branch of the mux has more inputs than this one,
		 * and the selected input index is not a valid index for
		 * this portion of the mux.  If the mux was actually built
		 * this way, their would likely be a weak pull-up to ensure
		 * that the node does not float.
		 */
		*out_prob = 1.0;
		*out_dens = 0.0;

		v_in_selected = 0.;
		for (input_idx = 0; input_idx < num_inputs; input_idx++) {
			v_in_selected += v_in[input_idx];
		}
		v_in_selected /= num_inputs;
	}

	power_calc_transistor_capacitance(&C_drain, &C_source, &C_gate, NMOS, 1.0,
			POWER_CIRCUIT_TYPE_PASS);

	in_prob_avg = 0.;
	for (input_idx = 0; input_idx < num_inputs; input_idx++) {
		/* Dynamic Power at Inputs */
		power_usage->dynamic += power_calc_dynamic_v(C_drain,
				in_dens[input_idx], v_in[input_idx]);

		if (input_idx != selected_idx) {
			in_prob_avg += in_prob[input_idx];
		}
	}
	in_prob_avg /= (num_inputs - 1);

	if (v_out_restored) {
		*v_out = g_power_tech->Vdd;
	} else {
		*v_out = power_calc_mux_v_out(num_inputs, transistor_size,
				v_in_selected, in_prob_avg);
	}

	for (input_idx = 0; input_idx < num_inputs; input_idx++) {
		/* Leakage */
		/* The selected input will never leak */
		if (input_idx == selected_idx) {
			continue;
		}

		/* Output is high and this input is low */
		power_usage->leakage += (*out_prob) * (1 - in_prob[input_idx])
				* power_calc_leakage_pass_transistor(transistor_size, *v_out);

		/* Output is low and this input is high */
		power_usage->leakage += (1 - *out_prob) * in_prob[input_idx]
				* power_calc_leakage_pass_transistor(transistor_size,
						v_in[input_idx]);
	}

	/* Dynamic Power at Output */
	power_usage->dynamic += power_calc_dynamic_v(C_source * num_inputs,
			*out_dens, *v_out);

}

float power_calc_mux_v_out(int num_inputs, float transistor_size, float v_in,
		float in_prob_avg) {
	t_power_mux_volt_inf * mux_volt_inf;
	t_power_mux_volt_pair * lower;
	t_power_mux_volt_pair * upper;
	float v_out_min, v_out_max;

	assert(transistor_size = 1.0);

	if (num_inputs > g_power_tech->max_mux_sl_size) {
		power_log_msg(POWER_LOG_ERROR,
				"The circuit contains a single-level mux larger than \
				what is defined in the <multiplexers> section of the \
				technology file.");
	}

	mux_volt_inf = &g_power_tech->mux_voltage_inf[num_inputs];

	power_find_mux_volt_inf(&lower, &upper, mux_volt_inf, v_in);

	if (lower->v_in == v_in) {
		v_out_min = lower->v_out_min;
		v_out_max = lower->v_out_max;
	} else {
		float perc_upper = (v_in - lower->v_in) / (upper->v_in - lower->v_in);
		v_out_min = (1 - perc_upper) * lower->v_out_min
				+ perc_upper * upper->v_out_min;
		v_out_max = (1 - perc_upper) * lower->v_out_max
				+ perc_upper * upper->v_out_max;
	}

	return in_prob_avg * v_out_max + (1 - in_prob_avg) * v_out_min;
}

void power_calc_mux_singlelevel_dynamic(t_power_usage * power_usage,
		int num_inputs, float out_density, float out_prob, float v_out,
		float * in_prob, float * in_density, float * v_in, float sel_dens,
		float sel_prob, float transistor_size) {

	assert(num_inputs == 2);
	assert(transistor_size = 1.0);

	power_zero_usage(power_usage);

	/* Leakage occurs when input1 != input2.
	 * If the selected input is low, the other transistor leaks input->output
	 * If the selected input is high, the other transistor leaks output->input*/

	/* 1st selected, 1st Low, 2nd High - Leakage from 2nd in->out */
	power_usage->leakage += (1 - sel_prob) * (1 - in_prob[0]) * in_prob[1]
			* power_calc_leakage_pass_transistor(1.0, v_in[1]);

	/* 1st selected, 1st High, 2nd Low - Leakage from 2nd out->in */
	/* 2nd selected, 1st Low, 2nd High - Leakage from 1st out->in */
	power_usage->leakage += ((1 - sel_prob) * in_prob[0] * (1 - in_prob[1])
			+ sel_prob * (1 - in_prob[0]) * in_prob[1])
			* power_calc_leakage_pass_transistor(1.0, v_out);

	/* 2nd selected, 1st High, 2nd Low - Leakage from 1st in->out */
	power_usage->leakage += sel_prob * in_prob[0] * (1 - in_prob[1])
			* power_calc_leakage_pass_transistor(1.0, v_in[0]);

	/* Gate switching */
	power_usage->dynamic += 2
			* power_calc_dynamic(g_power_commonly_used->NMOS_1X_C_gate_pass,
					sel_dens);

	/* Input switching */
	power_usage->dynamic += power_calc_dynamic_v(
			g_power_commonly_used->NMOS_1X_C_drain_pass, in_density[0],
			v_in[0]);
	power_usage->dynamic += power_calc_dynamic_v(
			g_power_commonly_used->NMOS_1X_C_drain_pass, in_density[1],
			v_in[1]);

	/* Output switching */
	power_usage->dynamic += power_calc_dynamic_v(
			2 * g_power_commonly_used->NMOS_1X_C_source_pass, out_density,
			v_out);
}

void power_calc_level_restorer(t_power_usage * power_usage, float * dyn_power_in, float in_density,
		float in_probability) {
	t_power_usage sub_power_usage;
	float C;
	float C_in;
	float input_dyn_power = 0.;

	power_zero_usage(power_usage);

	/* Inverter */
	power_calc_inverter_irregular(&sub_power_usage, &input_dyn_power, in_density, in_probability,
			1.0, 2.0);
	power_add_usage(power_usage, &sub_power_usage);

	if (g_power_tech->PMOS_inf.long_trans_inf == NULL ) {
		power_log_msg(POWER_LOG_ERROR,
				"No long transistor information exists.  Cannot determine transistor properties.");
		return;
	}
	C = g_power_tech->PMOS_inf.long_trans_inf->C_drain_cmos
			+ g_power_tech->PMOS_inf.long_trans_inf->C_gate_cmos;
	C_in = g_power_tech->PMOS_inf.long_trans_inf->C_drain_cmos;

	input_dyn_power += power_calc_dynamic(C_in, in_density);
	power_usage->dynamic += power_calc_dynamic(C, in_density);
	power_usage->leakage += (1 - in_probability)
			* g_power_tech->PMOS_inf.long_trans_inf->leakage_subthreshold;

	*dyn_power_in = input_dyn_power;
}

float power_calc_pb_dyn_from_c_internal(t_pb * pb,
		t_pb_graph_node * pb_graph_node) {
	float density = 0.;

	int port_idx;
	int pin_idx;
	int num_pins = 0;
	for (port_idx = 0; port_idx < pb_graph_node->num_input_ports; port_idx++) {
		for (pin_idx = 0; pin_idx < pb_graph_node->num_input_pins[port_idx];
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
	return power_calc_dynamic(pb_graph_node->pb_type->C_internal, density);
}

static float power_calc_buffer_sc_levr(t_power_buffer_strength_inf * buffer_strength,
		int input_mux_size) {
	t_power_buffer_sc_levr_inf * mux_lower;
	t_power_buffer_sc_levr_inf * mux_upper;

	power_find_buffer_sc_levr(&mux_lower, &mux_upper, buffer_strength,
			input_mux_size);
	if (mux_upper == NULL ) {
		return mux_lower->sc_levr;
	} else {
		float percent_upper = (input_mux_size - mux_lower->mux_size)
				/ (mux_upper->mux_size - mux_lower->mux_size);
		return (1 - percent_upper) * mux_lower->sc_levr
				+ percent_upper * mux_upper->sc_levr;
	}
}

float power_calc_buffer_sc(int stages, float gain, boolean level_restored,
		int input_mux_size) {

	t_power_buffer_size_inf * size_inf;
	t_power_buffer_strength_inf * strength_lower;
	t_power_buffer_strength_inf * strength_upper;
	float sc;

	size_inf = & g_power_tech->buffer_size_inf[stages];


	power_find_buffer_strength_inf(&strength_lower, &strength_upper, size_inf, gain);

	if (!level_restored) {
		if (strength_upper == NULL ) {
			sc = strength_lower->sc_no_levr;
		} else {
			float percent_upper = (gain - strength_lower->stage_gain)
					/ (strength_upper->stage_gain - strength_lower->stage_gain);
			sc = (1 - percent_upper) * strength_lower->sc_no_levr
					+ percent_upper * strength_upper->sc_no_levr;
		}
	} else {
		/* Level Restored - Short Circuit depends on input mux size */

		if (strength_upper == NULL ) {
			sc = power_calc_buffer_sc_levr(strength_lower, input_mux_size);
		} else {
			float sc_buf_low;
			float sc_buf_high;

			sc_buf_low = power_calc_buffer_sc_levr(strength_lower, input_mux_size);
			sc_buf_high = power_calc_buffer_sc_levr(strength_upper, input_mux_size);

			float percent_upper = (gain - strength_lower->stage_gain)
					/ (strength_upper->stage_gain - strength_lower->stage_gain);
			sc = (1 - percent_upper) * sc_buf_low + percent_upper * sc_buf_high;
		}
	}
	return sc;
}
