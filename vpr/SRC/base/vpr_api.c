/**
General API for VPR 
Other software tools should generally call just the functions defined here
For advanced/power users, you can call functions defined elsewhere in VPR or modify the data structures directly at your discretion but be aware that doing so can break the correctness of VPR

Author: Jason Luu
June 21, 2012
*/


#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <time.h>

#include "util.h"
#include "vpr_types.h"
#include "vpr_utils.h"
#include "globals.h"
#include "graphics.h"
#include "read_netlist.h"
#include "check_netlist.h"
#include "print_netlist.h"
#include "read_blif.h"
#include "draw.h"
#include "place_and_route.h"
#include "pack.h"
#include "SetupGrid.h"
#include "stats.h"
#include "path_delay.h"
#include "OptionTokens.h"
#include "ReadOptions.h"
#include "read_xml_arch_file.h"
#include "SetupVPR.h"
#include "rr_graph.h"
#include "pb_type_graph.h"
#include "ReadOptions.h"
#include "route_common.h"
#include "vpr_api.h"

/* Local subroutines */
static void free_pb_type(t_pb_type *pb_type);
static void free_complex_block_types(void);

static void free_arch(t_arch* Arch);
static void free_options(t_options *options);
static void free_circuit(void);
	
/* Local subroutines end */

/* Display general VPR information */
void vpr_print_title(void) {
	puts("");
	puts("VPR FPGA Placement and Routing.");
	puts("Version: Version " VPR_VERSION);
	puts("Compiled: " __DATE__ ".");
	puts("Original VPR by V. Betz.");
	puts("Timing-driven placement enhancements by A. Marquardt.");
	puts("Single-drivers enhancements by Andy Ye with additions by.");
	puts("Mark Fang, Jason Luu, Ted Campbell");
	puts("Heterogeneous stucture support by Jason Luu and Ted Campbell.");
	puts("T-VPack clustering integration by Jason Luu.");
	puts("Area-driven AAPack added by Jason Luu.");
	puts("This is free open source code under MIT license.");
	puts("");

}

/* Display help screen */
void vpr_print_usage(void) {
	puts("Usage:  vpr fpga_architecture.xml circuit_name [Options ...]");
	puts("");
	puts("General Options:  [--nodisp] [--auto <int>] [--pack]");
	puts(
			"\t[--place] [--route] [--timing_analyze_only_with_net_delay <float>]");
	puts(
			"\t[--fast] [--full_stats] [--timing_analysis on | off] [--outfile_prefix <string>]");
	puts(
			"\t[--blif_file <string>][--net_file <string>][--place_file <string>]");
	puts("\t[--route_file <string>][--echo_file on | off]");
	puts("");
	puts("Packer Options:");
	/*    puts("\t[-global_clocks on|off]");
	 puts("\t[-hill_climbing on|off]");
	 puts("\t[-sweep_hanging_nets_and_inputs on|off]"); */
	puts("\t[--timing_driven_clustering on|off]");
	puts(
			"\t[--cluster_seed_type timing|max_inputs] [--alpha_clustering <float>] [--beta_clustering <float>]");
	/*    puts("\t[-recompute_timing_after <int>] [-cluster_block_delay <float>]"); */
	puts("\t[--allow_unrelated_clustering on|off]");
	/*    puts("\t[-allow_early_exit on|off]"); 
	 puts("\t[-intra_cluster_net_delay <float>] ");
	 puts("\t[-inter_cluster_net_delay <float>] "); */
	puts("\t[--connection_driven_clustering on|off] ");
	puts("");
	puts("Placer Options:");
	puts(
			"\t[--place_algorithm bounding_box | net_timing_driven | path_timing_driven]");
	puts("\t[--init_t <float>] [--exit_t <float>]");
	puts("\t[--alpha_t <float>] [--inner_num <float>] [--seed <int>]");
	puts("\t[--place_cost_exp <float>]");
	puts("\t[--place_chan_width <int>] [--num_regions <int>] ");
	puts("\t[--fix_pins random | <file.pads>]");
	puts("\t[--enable_timing_computations on | off]");
	puts("\t[--block_dist <int>]");
	puts("");
	puts("Placement Options Valid Only for Timing-Driven Placement:");
	puts("\t[--timing_tradeoff <float>]");
	puts("\t[--recompute_crit_iter <int>]");
	puts("\t[--inner_loop_recompute_divider <int>]");
	puts("\t[--td_place_exp_first <float>]");
	puts("\t[--td_place_exp_last <float>]");
	puts("");
	puts("Router Options:  [-max_router_iterations <int>] [-bb_factor <int>]");
	puts("\t[--initial_pres_fac <float>] [--pres_fac_mult <float>]");
	puts("\t[--acc_fac <float>] [--first_iter_pres_fac <float>]");
	puts("\t[--bend_cost <float>] [--route_type global | detailed]");
	puts("\t[--verify_binary_search] [--route_chan_width <int>]");
	puts(
			"\t[--router_algorithm breadth_first | timing_driven | directed_search]");
	puts(
			"\t[--base_cost_type intrinsic_delay | delay_normalized | demand_only]");
	puts("");
	puts("Routing options valid only for timing-driven routing:");
	puts("\t[--astar_fac <float>] [--max_criticality <float>]");
	puts("\t[--criticality_exp <float>]");
	puts("");
}

/* Initialize VPR 
	1. Read Options
	2. Read Arch
	3. Read Circuit
	4. Sanity check all three
*/
void vpr_init(INP int argc, INP char **argv, OUTP t_options *options, OUTP t_vpr_setup *vpr_setup, OUTP t_arch *arch) {
	/* Print usage message if no args */
	if (argc < 3) {
		vpr_print_usage();
		exit(1);
	}

	memset(options, 0, sizeof(t_options));
	memset(vpr_setup, 0, sizeof(t_vpr_setup));
	memset(arch, 0, sizeof(t_arch));

	/* Read in user options */
	ReadOptions(argc, argv, options);
	/* Timing option priorities */
	vpr_setup->TimingEnabled = IsTimingEnabled(options);
	/* Determine whether echo is on or off */
	SetEchoOption(IsEchoEnabled(options));
	vpr_setup->constant_net_delay = options->constant_net_delay;

	/* Read in arch and circuit */
	SetupVPR(options, vpr_setup->TimingEnabled, &vpr_setup->FileNameOpts, arch, &vpr_setup->Operation,
			&vpr_setup->user_models, &vpr_setup->library_models, &vpr_setup->PackerOpts, &vpr_setup->PlacerOpts,
			&vpr_setup->AnnealSched, &vpr_setup->RouterOpts, &vpr_setup->RoutingArch, &vpr_setup->Segments, &vpr_setup->Timing,
			&vpr_setup->ShowGraphics, &vpr_setup->GraphPause);


	/* Check inputs are reasonable */
	CheckOptions(*options, vpr_setup->TimingEnabled);
	CheckArch(*arch, vpr_setup->TimingEnabled);

	/* Verify settings don't conflict or otherwise not make sense */
	CheckSetup(vpr_setup->Operation, vpr_setup->PlacerOpts, vpr_setup->AnnealSched, vpr_setup->RouterOpts, vpr_setup->RoutingArch,
			vpr_setup->Segments, vpr_setup->Timing, arch->Chans);

	/* flush any messages to user still in stdout that hasn't gotten displayed */
	fflush(stdout);

	/* Read blif file and sweep unused components */
	read_and_process_blif(vpr_setup->PackerOpts.blif_file_name, vpr_setup->PackerOpts.sweep_hanging_nets_and_inputs, vpr_setup->user_models, vpr_setup->library_models);
	fflush(stdout);

	ShowSetup(*options, *vpr_setup);
}


/* 
 * Sets globals: nx, ny
 * Allocs globals: chan_width_x, chan_width_y, grid
 * Depends on num_clbs, pins_per_clb */
void vpr_init_pre_place_and_route(INP t_vpr_setup vpr_setup, INP t_arch Arch) {
	int *num_instances_type, *num_blocks_type;
	int i;
	int current, high, low;
	boolean fit;

	/* Read in netlist file for placement and routing */
	if (vpr_setup.FileNameOpts.NetFile) {
		read_netlist(vpr_setup.FileNameOpts.NetFile, &Arch, &num_blocks, &block,
				&num_nets, &clb_net);
		/* This is done so that all blocks have subblocks and can be treated the same */
		check_netlist();
	}

	/* Output the current settings to console. */
	printClusteredNetlistStats();

	if (vpr_setup.Operation == TIMING_ANALYSIS_ONLY) {
		do_constant_net_delay_timing_analysis(vpr_setup.Timing, vpr_setup.constant_net_delay);
	} else {
		current = nint(sqrt(num_blocks)); /* current is the value of the smaller side of the FPGA */
		low = 1;
		high = -1;

		num_instances_type = (int*) my_calloc(num_types, sizeof(int));
		num_blocks_type = (int*) my_calloc(num_types, sizeof(int));

		for (i = 0; i < num_blocks; i++) {
			num_blocks_type[block[i].type->index]++;
		}

		if (Arch.clb_grid.IsAuto) {
			/* Auto-size FPGA, perform a binary search */
			while (high == -1 || low < high) {
				/* Generate grid */
				if (Arch.clb_grid.Aspect >= 1.0) {
					ny = current;
					nx = nint(current * Arch.clb_grid.Aspect);
				} else {
					nx = current;
					ny = nint(current / Arch.clb_grid.Aspect);
				}
	#if DEBUG
				printf("Auto-sizing FPGA, try x = %d y = %d\n", nx, ny);
	#endif
				alloc_and_load_grid(num_instances_type);
				freeGrid();

				/* Test if netlist fits in grid */
				fit = TRUE;
				for (i = 0; i < num_types; i++) {
					if (num_blocks_type[i] > num_instances_type[i]) {
						fit = FALSE;
						break;
					}
				}

				/* get next value */
				if (!fit) {
					/* increase size of max */
					if (high == -1) {
						current = current * 2;
						if (current > MAX_SHORT) {
							printf(
									ERRTAG
									"FPGA required is too large for current architecture settings\n");
							exit(1);
						}
					} else {
						if (low == current)
							current++;
						low = current;
						current = low + ((high - low) / 2);
					}
				} else {
					high = current;
					current = low + ((high - low) / 2);
				}
			}
			/* Generate grid */
			if (Arch.clb_grid.Aspect >= 1.0) {
				ny = current;
				nx = nint(current * Arch.clb_grid.Aspect);
			} else {
				nx = current;
				ny = nint(current / Arch.clb_grid.Aspect);
			}
			alloc_and_load_grid(num_instances_type);
			printf("FPGA auto-sized to, x = %d y = %d\n", nx, ny);
		} else {
			nx = Arch.clb_grid.W;
			ny = Arch.clb_grid.H;
			alloc_and_load_grid(num_instances_type);
		}

		printf("The circuit will be mapped into a %d x %d array of clbs.\n", nx,
				ny);

		/* Test if netlist fits in grid */
		fit = TRUE;
		for (i = 0; i < num_types; i++) {
			if (num_blocks_type[i] > num_instances_type[i]) {
				fit = FALSE;
				break;
			}
		}
		if (!fit) {
			printf(ERRTAG "Not enough physical locations for type %s, "
			"number of blocks is %d but number of locations is %d\n",
					type_descriptors[i].name, num_blocks_type[i],
					num_instances_type[i]);
			exit(1);
		}

		printf("\nResource Usage:\n");
		for (i = 0; i < num_types; i++) {
			printf("Netlist      %d\tblocks of type %s\n", num_blocks_type[i],
					type_descriptors[i].name);
			printf("Architecture %d\tblocks of type %s\n", num_instances_type[i],
					type_descriptors[i].name);
		}
		printf("\n");
		chan_width_x = (int *) my_malloc((ny + 1) * sizeof(int));
		chan_width_y = (int *) my_malloc((nx + 1) * sizeof(int));

		free(num_blocks_type);
		free(num_instances_type);
	}
}


void vpr_pack(INP t_vpr_setup vpr_setup, INP t_arch arch) {
	clock_t begin, end;

	begin = clock();
	printf("Initialize packing\n");
	try_pack(&vpr_setup.PackerOpts, &arch, vpr_setup.user_models, vpr_setup.library_models, vpr_setup.Timing);
	end = clock();
	#ifdef CLOCKS_PER_SEC
		printf("Packing took %g seconds\n",
					(float) (end - begin) / CLOCKS_PER_SEC);
		printf("Packing completed\n");
	#else
			printf("Packing took %g seconds\n", (float)(end - begin) / CLK_PER_SEC);
	#endif
	fflush(stdout);
}

void vpr_place_and_route(INP t_vpr_setup vpr_setup, INP t_arch arch) {
	/* Startup X graphics */
	set_graphics_state(vpr_setup.ShowGraphics, vpr_setup.GraphPause, vpr_setup.RouterOpts.route_type);
	if (vpr_setup.ShowGraphics) {
		init_graphics("VPR:  Versatile Place and Route for FPGAs");
		alloc_draw_structs();
	}

	/* Do placement and routing */
	place_and_route(vpr_setup.Operation, vpr_setup.PlacerOpts, vpr_setup.FileNameOpts.PlaceFile,
			vpr_setup.FileNameOpts.NetFile, vpr_setup.FileNameOpts.ArchFile, vpr_setup.FileNameOpts.RouteFile,
			vpr_setup.AnnealSched, vpr_setup.RouterOpts, vpr_setup.RoutingArch, vpr_setup.Segments, vpr_setup.Timing, arch.Chans,
			arch.models);

	fflush(stdout);

	/* Close down X Display */
	if (vpr_setup.ShowGraphics)
		close_graphics();
}

/* Free architecture data structures */
void free_arch(t_arch* Arch) {
	int i;
	t_model *model, *prev;
	t_model_ports *port, *prev_port;
	struct s_linked_vptr *vptr, *vptr_prev;

	freeGrid();
	free(chan_width_x);
	chan_width_x = NULL;
	free(chan_width_y);
	chan_width_y = NULL;

	for (i = 0; i < Arch->num_switches; i++) {
		if (Arch->Switches->name != NULL) {
			free(Arch->Switches[i].name);
		}
	}
	free(Arch->Switches);
	free(switch_inf);
	for (i = 0; i < Arch->num_segments; i++) {
		if (Arch->Segments->cb != NULL) {
			free(Arch->Segments[i].cb);
		}
		if (Arch->Segments->sb != NULL) {
			free(Arch->Segments[i].sb);
		}
	}
	free(Arch->Segments);
	model = Arch->models;
	while (model) {
		port = model->inputs;
		while (port) {
			prev_port = port;
			port = port->next;
			free(prev_port->name);
			free(prev_port);
		}
		port = model->outputs;
		while (port) {
			prev_port = port;
			port = port->next;
			free(prev_port->name);
			free(prev_port);
		}
		vptr = model->pb_types;
		while (vptr) {
			vptr_prev = vptr;
			vptr = vptr->next;
			free(vptr_prev);
		}
		prev = model;

		model = model->next;
		if (prev->instances)
			free(prev->instances);
		free(prev->name);
		free(prev);
	}

	for (i = 0; i < 4; i++) {
		vptr = Arch->model_library[i].pb_types;
		while (vptr) {
			vptr_prev = vptr;
			vptr = vptr->next;
			free(vptr_prev);
		}
	}

	free(Arch->model_library[0].name);
	free(Arch->model_library[0].outputs->name);
	free(Arch->model_library[0].outputs);
	free(Arch->model_library[1].inputs->name);
	free(Arch->model_library[1].inputs);
	free(Arch->model_library[1].name);
	free(Arch->model_library[2].name);
	free(Arch->model_library[2].inputs[0].name);
	free(Arch->model_library[2].inputs[1].name);
	free(Arch->model_library[2].inputs);
	free(Arch->model_library[2].outputs->name);
	free(Arch->model_library[2].outputs);
	free(Arch->model_library[3].name);
	free(Arch->model_library[3].inputs->name);
	free(Arch->model_library[3].inputs);
	free(Arch->model_library[3].outputs->name);
	free(Arch->model_library[3].outputs);
	free(Arch->model_library);

	free_complex_block_types();
	free_chunk_memory_trace();
}

void free_options(t_options *options) {
	free(options->ArchFile);
	free(options->CircuitName);
	if (options->BlifFile)
		free(options->BlifFile);
	if (options->NetFile)
		free(options->NetFile);
	if (options->PlaceFile)
		free(options->PlaceFile);
	if (options->RouteFile)
		free(options->RouteFile);
	if (options->out_file_prefix)
		free(options->out_file_prefix);
	if (options->PinFile)
		free(options->PinFile);
}

static void free_complex_block_types(void) {
	int i, j, k, m;

	free_all_pb_graph_nodes();

	for (i = 0; i < num_types; i++) {
		if (&type_descriptors[i] == EMPTY_TYPE) {
			continue;
		}
		free(type_descriptors[i].name);
		for (j = 0; j < type_descriptors[i].height; j++) {
			for (k = 0; k < 4; k++) {
				for (m = 0;
						m < type_descriptors[i].num_pin_loc_assignments[j][k];
						m++) {
					if (type_descriptors[i].pin_loc_assignments[j][k][m])
						free(type_descriptors[i].pin_loc_assignments[j][k][m]);
				}
				free(type_descriptors[i].pinloc[j][k]);
				free(type_descriptors[i].pin_loc_assignments[j][k]);
			}
			free(type_descriptors[i].pinloc[j]);
			free(type_descriptors[i].pin_loc_assignments[j]);
			free(type_descriptors[i].num_pin_loc_assignments[j]);
		}
		for(j = 0; j < type_descriptors[i].num_class; j++) {
			free(type_descriptors[i].class_inf[j].pinlist);
		}
		free(type_descriptors[i].pinloc);
		free(type_descriptors[i].pin_loc_assignments);
		free(type_descriptors[i].num_pin_loc_assignments);
		free(type_descriptors[i].pin_height);
		free(type_descriptors[i].class_inf);
		free(type_descriptors[i].is_global_pin);
		free(type_descriptors[i].pin_class);
		free(type_descriptors[i].grid_loc_def);

		free_pb_type(type_descriptors[i].pb_type);
		free(type_descriptors[i].pb_type);		
	}
	free(type_descriptors);
}

static void free_pb_type(t_pb_type *pb_type) {	
	int i, j, k, m;

	free(pb_type->name);
	if (pb_type->blif_model)
		free(pb_type->blif_model);

	for (i = 0; i < pb_type->num_modes; i++) {
		for (j = 0; j < pb_type->modes[i].num_pb_type_children; j++) {
			free_pb_type(&pb_type->modes[i].pb_type_children[j]);
		}
		free(pb_type->modes[i].pb_type_children);
		free(pb_type->modes[i].name);
		for (j = 0; j < pb_type->modes[i].num_interconnect; j++) {
			free(pb_type->modes[i].interconnect[j].input_string);
			free(pb_type->modes[i].interconnect[j].output_string);
			free(pb_type->modes[i].interconnect[j].name);

			for (k = 0; k < pb_type->modes[i].interconnect[j].num_annotations;
					k++) {
				if (pb_type->modes[i].interconnect[j].annotations[k].clock)
					free(
							pb_type->modes[i].interconnect[j].annotations[k].clock);
				if (pb_type->modes[i].interconnect[j].annotations[k].input_pins) {
					free(
							pb_type->modes[i].interconnect[j].annotations[k].input_pins);
				}
				if (pb_type->modes[i].interconnect[j].annotations[k].output_pins) {
					free(
							pb_type->modes[i].interconnect[j].annotations[k].output_pins);
				}
				for (m = 0;
						m
								< pb_type->modes[i].interconnect[j].annotations[k].num_value_prop_pairs;
						m++) {
					free(
							pb_type->modes[i].interconnect[j].annotations[k].value[m]);
				}
				free(pb_type->modes[i].interconnect[j].annotations[k].prop);
				free(pb_type->modes[i].interconnect[j].annotations[k].value);
			}
			free(pb_type->modes[i].interconnect[j].annotations);
		}
		if (pb_type->modes[i].interconnect)
			free(pb_type->modes[i].interconnect);
	}
	if (pb_type->modes)
		free(pb_type->modes);

	for (i = 0; i < pb_type->num_annotations; i++) {
		for (j = 0; j < pb_type->annotations[i].num_value_prop_pairs; j++) {
			free(pb_type->annotations[i].value[j]);
		}
		free(pb_type->annotations[i].value);
		free(pb_type->annotations[i].prop);
		if (pb_type->annotations[i].input_pins) {
			free(pb_type->annotations[i].input_pins);
		}
		if (pb_type->annotations[i].output_pins) {
			free(pb_type->annotations[i].output_pins);
		}
		if (pb_type->annotations[i].clock) {
			free(pb_type->annotations[i].clock);
		}
	}
	if (pb_type->num_annotations > 0) {
		free(pb_type->annotations);
	}

	for (i = 0; i < pb_type->num_ports; i++) {
		free(pb_type->ports[i].name);
		if (pb_type->ports[i].port_class) {
			free(pb_type->ports[i].port_class);
		}
	}
	free(pb_type->ports);
}


void free_circuit() {
	int i;

	/* Free netlist reference tables for nets */
	free(clb_to_vpack_net_mapping);
	free(vpack_to_clb_net_mapping);
	clb_to_vpack_net_mapping = NULL;
	vpack_to_clb_net_mapping = NULL;


	/* Free logical blocks and nets */
	if (logical_block != NULL) {
		free_logical_blocks();
		free_logical_nets();
	}

	if(clb_net != NULL) {
		for(i = 0; i <  num_nets; i++) {
			free(clb_net[i].name);
			free(clb_net[i].node_block);
			free(clb_net[i].node_block_pin);
			free(clb_net[i].node_block_port);
		}
	}
	free(clb_net);
	clb_net = NULL;

	if(block != NULL) {
		for(i = 0; i < num_blocks; i++) {
			if(block[i].pb != NULL) {
				free_cb(block[i].pb);
				free(block[i].pb);
			}
			free(block[i].nets);
			free(block[i].name);
		}
	}
	free(block);
	block = NULL;

	free(blif_circuit_name);
	blif_circuit_name = NULL;

}

void vpr_free_all(INOUTP t_arch Arch, INOUTP t_options options, INOUTP t_vpr_setup vpr_setup) {
	
	if(vpr_setup.Timing.SDCFile != NULL) {
		free(vpr_setup.Timing.SDCFile);
		vpr_setup.Timing.SDCFile = NULL;
	}

	free_options(&options);
	free_circuit();
	free_arch(&Arch);
}





