/************************* INCLUDES *********************************/
#include <string.h>
#include <assert.h>

#include "power_components.h"
#include "power_lowlevel.h"
#include "power_util.h"

/************************* STRUCTS **********************************/
typedef struct s_power_breakdown t_power_breakdown;

struct s_power_breakdown {
	t_power_usage * components;
};

/************************* GLOBALS **********************************/
static t_power_breakdown * g_power_breakdown;

/************************* FUNCTION DECLARATIONS ********************/
static void power_component_print_usage_rec(FILE * fp, char * name,
		e_power_component_type type, e_power_component_type parent_type,
		int intdent_level);

static void power_calc_mux_rec(t_power_usage * power_usage, float * out_prob,
		float * out_dens, float * v_out, t_mux_node * mux_node,
		t_mux_arch * mux_arch, int * selector_values,
		float * primary_input_prob, float * primary_input_dens,
		boolean v_out_restored);

/************************* FUNCTION DEFINITIONS *********************/
void power_components_init(void) {
	int i;

	g_power_breakdown = my_malloc(sizeof(t_power_breakdown));
	g_power_breakdown->components = my_calloc(POWER_COMPONENT_MAX_NUM,
			sizeof(t_power_usage));
	for (i = 0; i < POWER_COMPONENT_MAX_NUM; i++) {
		power_zero_usage(&g_power_breakdown->components[i]);
	}
}

void power_components_uninit(void) {
	free(g_power_breakdown->components);
	free(g_power_breakdown);
}

void power_component_add_usage(t_power_usage * power_usage,
		e_power_component_type component_idx) {
	power_add_usage(&g_power_breakdown->components[component_idx], power_usage);
}

void power_component_get_usage(t_power_usage * power_usage,
		e_power_component_type component_idx) {
	memcpy(power_usage, &g_power_breakdown->components[component_idx],
			sizeof(t_power_usage));
}

float power_component_get_usage_sum(e_power_component_type component_idx) {
	return power_usage_sum(&g_power_breakdown->components[component_idx]);
}

void power_component_print_usage(FILE * fp) {
	power_component_print_usage_rec(fp, "Total", POWER_COMPONENT_TOTAL,
			POWER_COMPONENT_TOTAL, 0);
}

static void power_component_print_usage_rec(FILE * fp, char * name,
		e_power_component_type type, e_power_component_type parent_type,
		int indent_level) {
	int i;

	const int component_str_size = 30;

	if (type == POWER_COMPONENT_TOTAL) {
		fprintf(fp, "%-*s%-20s%-15s%-15s%-15s\n", component_str_size,
				"Component", "Power (W)", "%-Parent", "%-Total", "%-Dynamic");
	}

	for (i = 0; i < indent_level; i++) {
		fprintf(fp, "  ");
	}

	fprintf(fp, "%-*s%-20.4g%-15.3g%-15.3g%-15.3g\n",
			component_str_size - 2 * indent_level, name,
			power_usage_sum(&g_power_breakdown->components[type]),
			power_usage_sum(&g_power_breakdown->components[type])
					/ power_usage_sum(
							&g_power_breakdown->components[parent_type]),
			power_usage_sum(&g_power_breakdown->components[type])
					/ power_usage_sum(
							&g_power_breakdown->components[POWER_COMPONENT_TOTAL]),
			g_power_breakdown->components[type].dynamic
					/ power_usage_sum(&g_power_breakdown->components[type]));

	switch (type) {
	case (POWER_COMPONENT_TOTAL):
		power_component_print_usage_rec(fp, "Routing", POWER_COMPONENT_ROUTING,
				type, indent_level + 1);
		power_component_print_usage_rec(fp, "Clock", POWER_COMPONENT_CLOCK,
				type, indent_level + 1);
		power_component_print_usage_rec(fp, "CLBs", POWER_COMPONENT_CLB, type,
				indent_level + 1);
		break;
	case (POWER_COMPONENT_ROUTING):
		power_component_print_usage_rec(fp, "Switchbox",
				POWER_COMPONENT_ROUTE_SB, type, indent_level + 1);
		power_component_print_usage_rec(fp, "Connectionbox",
				POWER_COMPONENT_ROUTE_CB, type, indent_level + 1);
		power_component_print_usage_rec(fp, "GlobalWires",
				POWER_COMPONENT_ROUTE_GLB_WIRE, type, indent_level + 1);
		break;
	case (POWER_COMPONENT_CLOCK):
		power_component_print_usage_rec(fp, "ClockBuffer",
				POWER_COMPONENT_CLOCK_BUFFER, type, indent_level + 1);
		power_component_print_usage_rec(fp, "ClockWire",
				POWER_COMPONENT_CLOCK_WIRE, type, indent_level + 1);
		break;
	case (POWER_COMPONENT_CLB):
		power_component_print_usage_rec(fp, "Interconnect",
				POWER_COMPONENT_LOCAL_INTERC, type, indent_level + 1);
		power_component_print_usage_rec(fp, "LocalWire",
				POWER_COMPONENT_LOCAL_WIRE, type, indent_level + 1);
		power_component_print_usage_rec(fp, "FF", POWER_COMPONENT_FF, type,
				indent_level + 1);
		power_component_print_usage_rec(fp, "LUT", POWER_COMPONENT_LUT, type,
				indent_level + 1);
		break;
	case (POWER_COMPONENT_LUT):

		power_component_print_usage_rec(fp, "Driver",
				POWER_COMPONENT_LUT_DRIVER, type, indent_level + 1);
		power_component_print_usage_rec(fp, "Mux", POWER_COMPONENT_LUT_MUX,
				type, indent_level + 1);
		power_component_print_usage_rec(fp, "Restorer",
				POWER_COMPONENT_LUT_RESTORER, type, indent_level + 1);

		break;
	default:
		break;
	}

	if (type == POWER_COMPONENT_TOTAL) {
		fprintf(g_power_output->out, "\n");
	}
}

void power_calc_FF(t_power_usage * power_usage, float D_prob, float D_dens,
		float Q_prob, float Q_dens, float clk_prob, float clk_dens) {
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

	power_calc_inverter(&sub_power_usage, (1 - clk_prob) * D_dens, D_prob, 1.0);
	power_add_usage(power_usage, &sub_power_usage);

	power_calc_inverter(&sub_power_usage, (1 - clk_prob) * D_dens, 1 - D_prob,
			1.0);
	power_add_usage(power_usage, &sub_power_usage);

	/* Slave */
	mux_in_dens[0] = Q_dens;
	mux_in_dens[1] = (1 - clk_prob) * D_dens;
	mux_in_prob[0] = (1 - Q_prob);
	mux_in_prob[1] = (1 - D_prob);
	power_calc_MUX2_transmission(&sub_power_usage, mux_in_dens, mux_in_prob,
			clk_dens, clk_prob, Q_dens);
	power_add_usage(power_usage, &sub_power_usage);

	power_calc_inverter(&sub_power_usage, Q_dens, 1 - Q_prob, 1.0);
	power_add_usage(power_usage, &sub_power_usage);

	power_calc_inverter(&sub_power_usage, Q_dens, Q_prob, 1.0);
	power_add_usage(power_usage, &sub_power_usage);

	power_component_add_usage(power_usage, POWER_COMPONENT_FF);
	return;
}

void power_calc_LUT(t_power_usage * power_usage, int LUT_size,
		char * SRAM_values, float * input_prob, float * input_dens) {
	float **internal_prob;
	float **internal_dens;
	float **internal_v;
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
	internal_dens = my_calloc(LUT_size + 1, sizeof(float*));
	internal_v = my_calloc(LUT_size + 1, sizeof(float*));
	for (i = 0; i <= LUT_size; i++) {
		internal_prob[i] = my_calloc(1 << (LUT_size - i), sizeof(float));
		internal_dens[i] = my_calloc(1 << (LUT_size - i), sizeof(float));
		internal_v[i] = my_calloc(1 << (LUT_size - i), sizeof(float));
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
		internal_dens[0][i] = 0.;
		internal_v[0][i] = g_power_tech->Vdd;
	}

	power_per_lut = 0.0;
	input_related_power = 0.0;

	for (level_idx = 0; level_idx < LUT_size; level_idx++) {
		t_power_usage driver_power_usage;
		int MUXs_this_level;
		int MUX_idx;
		int reverse_idx = LUT_size - level_idx - 1;

		MUXs_this_level = 1 << (reverse_idx);

		power_calc_inverter(&driver_power_usage, input_dens[reverse_idx],
				input_prob[reverse_idx], 1.0);
		power_add_usage(power_usage, &driver_power_usage);
		power_component_add_usage(&driver_power_usage,
				POWER_COMPONENT_LUT_DRIVER);

		power_calc_inverter(&driver_power_usage, input_dens[reverse_idx],
				input_prob[reverse_idx], 2.0);
		power_add_usage(power_usage, &driver_power_usage);
		power_component_add_usage(&driver_power_usage,
				POWER_COMPONENT_LUT_DRIVER);

		power_calc_inverter(&driver_power_usage, input_dens[reverse_idx],
				1 - input_prob[reverse_idx], 2.0);
		power_add_usage(power_usage, &driver_power_usage);
		power_component_add_usage(&driver_power_usage,
				POWER_COMPONENT_LUT_DRIVER);
		/*
		 power_calc_INV1(&driver_power_usage, input_densities[reverse_idx],
		 input_probabilities[reverse_idx]);
		 power_add_usage_element(power_usage, &driver_power_usage,
		 POWER_COMPONENT_LUT_DRIVER);

		 power_calc_INV2(&driver_power_usage, input_densities[reverse_idx],
		 input_probabilities[reverse_idx]);
		 power_add_usage_element(power_usage, &driver_power_usage,
		 POWER_COMPONENT_LUT_DRIVER);

		 power_calc_INV2(&driver_power_usage, input_densities[reverse_idx],
		 1 - input_probabilities[reverse_idx]);
		 power_add_usage_element(power_usage, &driver_power_usage,
		 POWER_COMPONENT_LUT_DRIVER);
		 */

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
		/*
		 if (level_restorer_this_level) {
		 Vout_level = POWER_VOLTAGE_LEVEL_VDD;
		 } else {
		 Vout_level = POWER_VOLTAGE_LEVEL_VDD_SUB_VTH;
		 }

		 if (level_restorer_last_level) {
		 Vin_level = POWER_VOLTAGE_LEVEL_VDD;
		 } else {
		 Vin_level = POWER_VOLTAGE_LEVEL_VDD_SUB_VTH;
		 }*/

		for (MUX_idx = 0; MUX_idx < MUXs_this_level; MUX_idx++) {
			t_power_usage sub_power;
			float out_prob;
			float out_dens;
			float sum_prob = 0;
			int sram_offset = MUX_idx * pow(2, level_idx + 1);
			int sram_per_branch = pow(2, level_idx);
			int branch_lvl_idx;
			int sram_idx;
			float v_out;

			/* Calculate output probability of multiplexer */
			out_prob = internal_prob[level_idx][MUX_idx * 2]
					* (1 - input_prob[reverse_idx])
					+ internal_prob[level_idx][MUX_idx * 2 + 1]
							* input_prob[reverse_idx];

			/* Calculate output density of multiplexer */
			out_dens = internal_dens[level_idx][MUX_idx * 2]
					* (1 - input_prob[reverse_idx])
					+ internal_dens[level_idx][MUX_idx * 2 + 1]
							* input_prob[reverse_idx];

#ifdef POWER_LUT_FAST
			out_dens += ((1 - internal_prob[level_idx][MUX_idx * 2]) * internal_prob[level_idx][MUX_idx * 2 + 1]
					+ internal_prob[level_idx][MUX_idx * 2] * (1 - internal_prob[level_idx][MUX_idx * 2 + 1]))
			* input_dens[reverse_idx];
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
						branch_prob *= (1 - input_prob[branch_lvl_reverse_idx]);
					} else {
						branch_prob *= input_prob[branch_lvl_reverse_idx];
					}
				}
				sum_prob += branch_prob;
			}
			out_dens += sum_prob * input_dens[reverse_idx];
#endif

			/* Calculate output voltage of multiplexer */
			if (level_restorer_this_level) {
				v_out = g_power_tech->Vdd;
			} else {
				v_out = (1 - input_prob[reverse_idx])
						* power_calc_mux_v_out(2, 1.0,
								internal_v[level_idx][MUX_idx * 2],
								internal_prob[level_idx][MUX_idx * 2 + 1])
						+ input_prob[reverse_idx]
								* power_calc_mux_v_out(2, 1.0,
										internal_v[level_idx][MUX_idx * 2 + 1],
										internal_prob[level_idx][MUX_idx * 2]);
			}

			internal_dens[level_idx + 1][MUX_idx] = out_dens;
			internal_prob[level_idx + 1][MUX_idx] = out_prob;
			internal_v[level_idx + 1][MUX_idx] = v_out;

			power_calc_mux_singlelevel_dynamic(&sub_power, 2,
					internal_dens[level_idx + 1][MUX_idx],
					internal_prob[level_idx + 1][MUX_idx],
					internal_v[level_idx + 1][MUX_idx],
					&internal_prob[level_idx][MUX_idx * 2],
					&internal_dens[level_idx][MUX_idx * 2],
					&internal_v[level_idx][MUX_idx * 2],
					input_dens[reverse_idx], input_prob[reverse_idx], 1.0);
			power_add_usage(power_usage, &sub_power);
			power_component_add_usage(&sub_power, POWER_COMPONENT_LUT_MUX);

			if (level_restorer_this_level) {
				/* Level restorer */
				power_calc_buffer(&sub_power, 1,
						internal_prob[level_idx + 1][MUX_idx],
						internal_dens[level_idx + 1][MUX_idx], TRUE, 2);
				/*power_calc_level_restorer(&sub_power,
				 internal_dens[level_idx + 1][MUX_idx],
				 internal_prob[level_idx + 1][MUX_idx]);*/
				power_add_usage(power_usage, &sub_power);
				power_component_add_usage(&sub_power,
						POWER_COMPONENT_LUT_RESTORER);
			}
		}

	}

	for (i = 0; i <= LUT_size; i++) {
		free(internal_prob[i]);
		free(internal_dens[i]);
		free(internal_v[i]);
	}
	free(internal_prob);
	free(internal_dens);
	free(internal_v);

	power_component_add_usage(power_usage, POWER_COMPONENT_LUT);

	return;
}

void power_calc_interconnect(t_power_usage * power_usage, t_pb * pb,
		t_interconnect_pins * interc_pins, float interc_length) {
	int pin_idx;
	int out_port_idx;
	int in_port_idx;
	float * in_dens;
	float * in_prob;
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
				power_calc_wire(&sub_power_usage, C_wire, dens);
				power_add_usage(power_usage, &sub_power_usage);
				power_component_add_usage(&sub_power_usage,
						POWER_COMPONENT_LOCAL_WIRE);
			}

			/* Wire from output */
			dens = pin_density(pb,
					interc_pins->output_pins[out_port_idx][pin_idx]);
			power_calc_wire(&sub_power_usage, C_wire, dens);
			power_add_usage(power_usage, &sub_power_usage);
			power_component_add_usage(&sub_power_usage,
					POWER_COMPONENT_LOCAL_WIRE);
		}
	}

	switch (interc_pins->interconnect->type) {
	case DIRECT_INTERC:
		break;
	case MUX_INTERC:
	case COMPLETE_INTERC:
		in_dens = my_calloc(interc->num_input_ports, sizeof(float));
		in_prob = my_calloc(interc->num_input_ports, sizeof(float));

		for (out_port_idx = 0; out_port_idx < interc->num_output_ports;
				out_port_idx++) {
			for (pin_idx = 0; pin_idx < interc->num_pins_per_port; pin_idx++) {
				t_power_usage MUX_power;
				int selected_input = OPEN;

				/* Clear input densities */
				for (in_port_idx = 0; in_port_idx < interc->num_input_ports;
						in_port_idx++) {
					in_dens[in_port_idx] = 0.;
					in_prob[in_port_idx] = 0.;
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
								in_dens[in_port_idx] = pin_density(pb,
										input_pin);
								in_prob[in_port_idx] = pin_prob(pb, input_pin);
							}
						}

						/* Check that the input pin was found with a matching net to the output pin */
						assert(selected_input != OPEN);
					}
				} else {
					selected_input = 0;
				}

				power_calc_mux_multilevel(&MUX_power,
						interc_pins->interconnect->mux_arch, in_prob, in_dens,
						selected_input, TRUE);

				power_add_usage(power_usage, &MUX_power);
				power_component_add_usage(&MUX_power,
						POWER_COMPONENT_LOCAL_INTERC);
			}
		}

		free(in_dens);
		free(in_prob);
		break;
	}

	power_add_usage(&interc_pins->interconnect->power_usage, power_usage);
}

void power_calc_mux_multilevel(t_power_usage * power_usage,
		t_mux_arch * mux_arch, float * in_prob, float * in_dens,
		int selected_input, boolean output_level_restored) {
	float output_density;
	float output_prob;
	float V_out;
	boolean found;
	int * selector_values = my_calloc(mux_arch->levels, sizeof(int));

	assert(selected_input != OPEN);

	power_zero_usage(power_usage);

	/* Find selection index at each level */

	found = mux_find_selector_values(selector_values, mux_arch->mux_graph_head,
			selected_input);
	assert(found);

	/* Calculate power of the multiplexor stages, from final stage, to first stages */
	power_calc_mux_rec(power_usage, &output_density, &output_prob, &V_out,
			mux_arch->mux_graph_head, mux_arch, selector_values, in_prob,
			in_dens, output_level_restored);

	free(selector_values);
}

static void power_calc_mux_rec(t_power_usage * power_usage, float * out_prob,
		float * out_dens, float * v_out, t_mux_node * mux_node,
		t_mux_arch * mux_arch, int * selector_values,
		float * primary_input_prob, float * primary_input_dens,
		boolean v_out_restored) {
	int input_idx;
	float * in_prob;
	float * in_dens;
	float * v_in;
	t_power_usage sub_power_usage;

	/* Single input mux is really just a wire, and has no power.
	 * Ensure that it has no children before returning. */
	if (mux_node->num_inputs == 1) {
		assert(mux_node->level == 0);
		return;
	}

	v_in = my_calloc(mux_node->num_inputs, sizeof(float));
	if (mux_node->level == 0) {
		/* First level of mux - inputs are primar inputs */
		in_prob = &primary_input_prob[mux_node->starting_pin_idx];
		in_dens = &primary_input_dens[mux_node->starting_pin_idx];

		for (input_idx = 0; input_idx < mux_node->num_inputs; input_idx++) {
			v_in[input_idx] = g_power_tech->Vdd;
		}
	} else {
		/* High level of mux - inputs recursive from lower levels */
		in_prob = my_calloc(mux_node->num_inputs, sizeof(float));
		in_dens = my_calloc(mux_node->num_inputs, sizeof(float));

		for (input_idx = 0; input_idx < mux_node->num_inputs; input_idx++) {
			power_calc_mux_rec(power_usage, &in_prob[input_idx],
					&in_dens[input_idx], &v_in[input_idx],
					&mux_node->children[input_idx], mux_arch, selector_values,
					primary_input_prob, primary_input_dens, FALSE);
		}
	}

	power_calc_mux_singlelevel_static(&sub_power_usage, out_prob, out_dens,
			v_out, mux_node->num_inputs, selector_values[mux_node->level],
			in_prob, in_dens, v_in, mux_arch->transistor_sizes[mux_node->level],
			v_out_restored);
	power_add_usage(power_usage, &sub_power_usage);

	if (mux_node->level != 0) {
		free(in_prob);
		free(in_dens);
	}
}

void power_calc_buffer(t_power_usage * power_usage, float size, float in_prob,
		float in_dens, boolean level_restored, int input_mux_size) {
	t_power_usage sub_power_usage;
	int i, num_stages;
	float stage_effort;
	float stage_inv_size;
	float stage_in_prob;
	float input_dyn_power;

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
			if (level_restored) {
				/* Sense Buffer */
				power_calc_level_restorer(&sub_power_usage, &input_dyn_power,
						in_dens, stage_in_prob);
			} else {
				power_calc_inverter_with_input(&sub_power_usage,
						&input_dyn_power, in_dens, stage_in_prob,
						stage_inv_size);
			}
		} else {
			power_calc_inverter(&sub_power_usage, in_dens, stage_in_prob,
					stage_inv_size);
		}
		power_add_usage(power_usage, &sub_power_usage);

		stage_in_prob = 1 - stage_in_prob;
	}

	/* SC add a factor to dynamic power, but the factor is not in addition to the input power
	 * Need to subtract input before adding factor - this matters for small buffers
	 */
	power_usage->dynamic += (power_usage->dynamic - input_dyn_power)
			* power_calc_buffer_sc(num_stages, stage_effort, level_restored,
					input_mux_size);
}

