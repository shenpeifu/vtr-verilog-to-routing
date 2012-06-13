#define DO_NOT_ANALYSE -1

/*********************** Defines for timing options *******************************/

#define CRITICALITY_DEF 1
/* Which definition of criticality should VPR use?  Possible values:
   #define CRITICALITY_DEF 1: criticality = 1 - (slack of this edge)/(maximum arrival time T_arr_max_global over any clock domain)
   #define CRITICALITY_DEF 2: criticality = 1 - (slack of this edge)/(arrival time T_arr_with_least_slack over path with least slack)
   #define CRITICALITY_DEF 3: criticality = maximum over all traversals of 1 - (slack of this edge for this traversal, if used)/(maximum arrival time T_arr_max_for_this_traversal for this traversal)									
*/

#define NORMALIZE_SLACKS
/* If defined, normalize slacks to be always positive in load_net_slack, except for the final, analysis pass. 
   Comment the above line out if you want the optimizers to work with actual, rather than normalized, slacks.
   See comments in load_net_slack in path_delay.c for further explanation. */
  
/**********************************************************************************/

float **alloc_and_load_timing_graph(t_timing_inf timing_inf);

float **alloc_and_load_pre_packing_timing_graph(float block_delay,
		float inter_cluster_net_delay, t_model *models, t_timing_inf timing_inf);

t_linked_int *allocate_and_load_critical_path(void);

void load_timing_graph_net_delays(float **net_delay);

void load_net_slack_and_criticality(float **net_slack, float **net_criticality, boolean do_lut_input_balancing, boolean is_final_analysis);

void free_timing_graph(float **net_slack);

void print_timing_graph(char *fname);

void print_net_slack(char *fname, float **net_slack);

void print_lut_remapping(char *fname);

void print_critical_path(char *fname);

void get_tnode_block_and_output_net(int inode, int *iblk_ptr, int *inet_ptr);

void do_constant_net_delay_timing_analysis(t_timing_inf timing_inf,
		float constant_net_delay_value);

void print_timing_graph_as_blif(char *fname, t_model *models);

extern int num_netlist_clocks; /* [0..num_netlist_clocks - 1] number of clocks in netlist */

extern t_clock * clock_list; /* [0..num_netlist_clocks - 1] array of clocks in netlist */

extern float ** timing_constraints; /* [0..num_netlist_clocks - 1 (source)][0..num_netlist_clocks - 1 (destination)] */