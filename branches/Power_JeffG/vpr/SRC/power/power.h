#ifndef __POWER_H__
#define __POWER_H__

#include "vpr_types.h"

/************************* DEFINES ***********************************/

#define MAX_LOGS 100000

#define OPERATING_TEMP_DEFAULT 25

#define JEFF_DEBUG 1

#define CLOCK_PROB 0.5
#define CLOCK_DENS 2

#define POWER_BUFFER_STAGE_GAIN 4.0

#define POWER_TRANSISTOR_SPACING_FACTOR 1.2

/************************* STRUCTS/ENUMS ****************************/
typedef enum {
	POWER_RET_CODE_SUCCESS = 0, POWER_RET_CODE_ERRORS, POWER_RET_CODE_WARNINGS
} e_power_ret_code;

/************************* GLOBALS **********************************/

/************************* FUNCTIONS ********************************/

boolean power_init(t_power_opts * power_opts, t_power_arch * power_arch);

boolean power_uninit(void);

e_power_ret_code power_total(void);

float power_calc_dynamic(float capacitance, float density);
float power_calc_dynamic_v(float capacitance, float density, float voltage);

void mux_arch_fix_levels(t_mux_arch * mux_arch);
t_mux_node * alloc_and_load_mux_graph(int num_inputs, int levels);

#endif /* __POWER_H__ */
