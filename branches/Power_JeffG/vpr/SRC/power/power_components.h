#ifndef __POWER_COMPONENTS_H__
#define __POWER_COMPONENTS_H__

/************************* INCLUDES *********************************/
#include "power.h"

/************************* Defines **********************************/
#define POWER_LUT_SLOW
#if (!(defined(POWER_LUT_SLOW) || defined(POWER_LUT_FAST)))
#define POWER_LUT_FAST
#endif

/************************* ENUMS ************************************/
typedef enum {
	POWER_COMPONENT_IGNORE = 0,
	POWER_COMPONENT_TOTAL,

	POWER_COMPONENT_ROUTING,
	POWER_COMPONENT_ROUTE_SB,
	POWER_COMPONENT_ROUTE_BUFFER,
	POWER_COMPONENT_ROUTE_CB,
	POWER_COMPONENT_ROUTE_GLB_WIRE,

	POWER_COMPONENT_CLOCK,
	POWER_COMPONENT_CLOCK_BUFFER,
	POWER_COMPONENT_CLOCK_WIRE,

	POWER_COMPONENT_CLB,
	POWER_COMPONENT_LOCAL_INTERC,
	POWER_COMPONENT_LOCAL_WIRE,
	POWER_COMPONENT_FF,
	POWER_COMPONENT_LUT,
	POWER_COMPONENT_LUT_DRIVER,
	POWER_COMPONENT_LUT_MUX,
	POWER_COMPONENT_LUT_RESTORER,

	POWER_COMPONENT_MAX_NUM
} e_power_component_type;

/************************* FUNCTION DECLARATIONS ********************/

void power_components_init(void);
void power_components_uninit(void);
void power_component_get_usage(t_power_usage * power_usage,
		e_power_component_type component_idx);
void power_component_add_usage(t_power_usage * power_usage,
		e_power_component_type component_idx);
float power_component_get_usage_sum(e_power_component_type component_idx);

void power_component_print_usage(FILE * fp);

void power_calc_FF(t_power_usage * power_usage, float D_prob, float D_dens,
		float Q_prob, float Q_dens, float clk_prob, float clk_dens);
void power_calc_LUT(t_power_usage * power_usage, int LUT_size,
		char * SRAM_values, float * input_densities,
		float * input_probabilities);
void power_calc_interconnect(t_power_usage * power_usage, t_pb * pb,
		t_interconnect_pins * interc_pins, float interc_length);
void power_calc_mux_multilevel(t_power_usage * power_usage,
		t_mux_arch * mux_arch, float * in_prob, float * in_dens,
		int selected_input, boolean output_level_restored);
void power_calc_buffer(t_power_usage * power_usage, float size, float in_prob,
		float in_dens, boolean level_restored, int input_mux_size);

#endif
