#ifndef READ_BLIF_H
#define READ_BLIF_H

#include "power.h"

void read_blif(char *blif_file, boolean sweep_hanging_nets_and_inputs,
		t_model *user_models, t_model *library_models,
		t_power_opts * power_opts);

void echo_input(char *blif_file, char *echo_file, t_model *library_models);

#endif
