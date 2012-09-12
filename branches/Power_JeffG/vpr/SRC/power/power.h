#ifndef __POWER_H__
#define __POWER_H__

/************************* INCLUDES *********************************/
#include "vpr_types.h"

/************************* DEFINES ***********************************/
#define MAX_LOGS 100000

#define OPERATING_TEMP_DEFAULT 25

#define JEFF_DEBUG 1

#define CLOCK_PROB 0.5
#define CLOCK_DENS 2

#define POWER_BUFFER_STAGE_GAIN 4.0

#define POWER_TRANSISTOR_SPACING_FACTOR 1.2

/************************* ENUMS ************************************/
typedef enum {
	POWER_RET_CODE_SUCCESS = 0, POWER_RET_CODE_ERRORS, POWER_RET_CODE_WARNINGS
} e_power_ret_code;

typedef enum {
	POWER_CIRCUIT_TYPE_CMOS = 0, POWER_CIRCUIT_TYPE_PASS
} e_power_circuit_type;

enum e_power_log_type {
	POWER_LOG_ERROR, POWER_LOG_WARNING, POWER_LOG_NUM_TYPES
};

/************************* STRUCTS **********************************/
typedef struct s_power_output t_power_output;
typedef struct s_log t_log;
typedef struct s_power_commonly_used t_power_commonly_used;
typedef struct s_power_mux_volt_inf t_power_mux_volt_inf;
typedef struct s_power_mux_volt_pair t_power_mux_volt_pair;
typedef struct s_power_nmos_leakages t_power_nmos_leakages;
typedef struct s_power_nmos_leakage_pair t_power_nmos_leakage_pair;
typedef struct s_power_buffer_sc_levr_inf t_power_buffer_sc_levr_inf;
typedef struct s_power_tech t_power_tech;
typedef struct s_transistor_inf t_transistor_inf;
typedef struct s_transistor_size_inf t_transistor_size_inf;
typedef struct s_power_opts t_power_opts;
typedef struct s_power_buffer_size_inf t_power_buffer_size_inf;
typedef struct s_power_buffer_strength_inf t_power_buffer_strength_inf;


struct s_power_opts {
	boolean do_power;
	char * activity_file;
	char * power_output_file;
	char * cmos_tech_behavior_file;
};

/* two types of transisters */
typedef enum {
	NMOS, PMOS
} e_tx_type;

/*
#define Q_charge 1.60e-19
#define vsat 8.00e4
#define Bolzmann_constant 1.38e-25 */



/*
 * Leakage High - Leakage for Vds = Vdd
 * Leakage Low - Leakage for Vds = Vdd - Vth
 */
struct s_transistor_size_inf {
	float size;

	float leakage_subthreshold;
	float leakage_gate;

	float C_gate_cmos;
	/*float C_source_cmos;*/
	float C_drain_cmos;
	float C_gate_pass;
	float C_source_pass;
	float C_drain_pass;
};



/* Record for storing the technology parameters for NMOS and
 PMOS type of transistors

 min_length: minimum channel width of the transistor (in m)
 min_width:  minimum width of the transistor (in m)
 Vth:        threshold voltage (in volt)
 CJ:         junction capacitance (F/m^2)
 CJSW:       side-wall junction capacitance (F/m)
 CJSWG:      gate-edge side-wall bulk junction capacitance (F/m)
 CGDO:       gate-drain overlap capacitance (F/m)
 COX:        gate-oxide cpacitance per unit area
 EC:         contant for leakage current calculation
 */

struct temperature_record {
	int temperature;
	float NMOS_NFS_value;
	float PMOS_NFS_value;
};
/* record for storing the NMOS and PMOS NFS values for leakage current
 calculation
 temperature: temperature in degree celcius
 NMOS_NFS_value: NFS value for NMOS for that temperature
 PMOS_NFS_value: NFS value for PMOS for that temperature
 */





/*
 For Power calculation of the clock network
 Rbuffer: Resistance of clock buffer
 Cbuffer_in: Input Capacitance of clock buffer
 Cbuffer_out: Output Capacitance of clock buffer
 Rwire: Wire resistance of the clock network (unit: ohm per segment length)
 Cwire: Wire capacitance of the clock network (unit: F per segment length)
 Cin_per_clb_clock_pin: input capacitable to each logic block clock pin
 clock_density: transition density for the clock (usually is 2)
 */







struct s_transistor_inf {
	/*float Vth;*/
	int num_size_entries;
	t_transistor_size_inf * size_inf;
	t_transistor_size_inf * long_trans_inf;
};

struct s_power_tech {
	float PN_ratio;
	float Vdd;

	float tech_size;
	float temperature;

	t_transistor_inf NMOS_inf;
	t_transistor_inf PMOS_inf;

	/* Pass Multiplexor Voltage Information */
	int max_mux_sl_size;
	t_power_mux_volt_inf * mux_voltage_inf;

	/* NMOS Leakage by Vds */
	int num_leakage_pairs;
	t_power_nmos_leakage_pair * leakage_pairs;

	/* Buffer Info */
	int max_buffer_size;
	t_power_buffer_size_inf * buffer_size_inf;
};

struct s_power_buffer_size_inf {
	int num_strengths;
	t_power_buffer_strength_inf * strength_inf;
};


/* Set of I/O Voltages for a single-level multiplexer */
struct s_power_mux_volt_inf {
	int num_voltage_pairs;
	t_power_mux_volt_pair * mux_voltage_pairs;
};

/* Single I/O voltage for a single-level multiplexer */
struct s_power_mux_volt_pair {
	float v_in;
	float v_out_min;
	float v_out_max;
};

struct s_power_buffer_strength_inf {
	float stage_gain;

	/* Short circuit factor - no level restorer */
	float sc_no_levr;

	/* Short circuit factors - level restorers */
	int num_levr_entries;
	t_power_buffer_sc_levr_inf * sc_levr_inf;
};

struct s_power_buffer_sc_levr_inf {
	int mux_size;

	/* Short circuit factor */
	float sc_levr;
};

struct s_power_nmos_leakage_pair {
	float v_ds;
	float i_ds;
};

struct s_power_output {
	FILE * out;
	t_log * logs;
	int num_logs;
};

struct s_log {
	char * name;
	char ** messages;
	int num_messages;
};

struct s_power_commonly_used {

	float NMOS_1X_C_gate_cmos;
	float NMOS_1X_C_drain_cmos;
	float NMOS_1X_C_source_cmos;
	float NMOS_1X_C_gate_pass;
	float NMOS_1X_C_drain_pass;
	float NMOS_1X_C_source_pass;
	float NMOS_1X_leakage_cmos;
	float NMOS_2X_leakage_cmos;
	float PMOS_1X_C_gate_cmos;
	float PMOS_1X_C_drain_cmos;
	float PMOS_1X_C_source_cmos;
	float PMOS_1X_C_gate_pass;
	float PMOS_1X_C_drain_pass;
	float PMOS_1X_C_source_pass;
	float PMOS_1X_leakage_cmos;
	float PMOS_2X_leakage_cmos;
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

	int num_sb_buffers;
	float total_sb_buffer_size;

	int num_cb_buffers;
	float total_cb_buffer_size;
};

/************************* GLOBALS **********************************/
extern t_power_arch * g_power_arch;
extern t_power_output * g_power_output;
extern t_power_commonly_used * g_power_commonly_used;
extern t_power_opts * g_power_opts;
extern t_power_tech * g_power_tech;

/************************* FUNCTION DECLARATIONS ********************/

boolean power_init(t_power_opts * power_opts, t_power_arch * power_arch);

boolean power_uninit(void);

e_power_ret_code power_total(void);

void mux_arch_fix_levels(t_mux_arch * mux_arch);
t_mux_node * alloc_and_load_mux_graph(int num_inputs, int levels);



#endif /* __POWER_H__ */
