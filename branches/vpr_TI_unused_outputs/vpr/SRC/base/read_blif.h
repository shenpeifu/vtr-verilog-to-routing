void read_blif (char *blif_file, 
		boolean sweep_hanging_nets_and_inputs,
/* JR-031412 */
		boolean ignore_unused_outputs,
		t_model *user_models, t_model *library_models);
void echo_input (char *blif_file, char *echo_file, t_model *library_models); 
