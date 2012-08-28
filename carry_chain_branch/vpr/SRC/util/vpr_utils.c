#include <assert.h>
#include <string.h>
#include "util.h"
#include "vpr_types.h"
#include "physical_types.h"
#include "globals.h"
#include "vpr_utils.h"
#include "cluster_placement.h"
#include "place_macro.h"
#include "string.h"

/* This module contains subroutines that are used in several unrelated parts *
 * of VPR.  They are VPR-specific utility routines.                          */

#define MAX_STRING_LEN 128

/******************** Subroutine definitions ********************************/

/**
 * print tabs given number of tabs to file
 */
void print_tabs(FILE * fpout, int num_tab) {
	int i;
	for (i = 0; i < num_tab; i++) {
		fprintf(fpout, "\t");
	}
}

/* Points the grid structure back to the blocks list */
void sync_grid_to_blocks(INP int L_num_blocks,
		INP const struct s_block block_list[], INP int L_nx, INP int L_ny,
		INOUTP struct s_grid_tile **L_grid) {
	int i, j, k;

	/* Reset usage and allocate blocks list if needed */
	for (j = 0; j <= (L_ny + 1); ++j) {
		for (i = 0; i <= (L_nx + 1); ++i) {
			L_grid[i][j].usage = 0;
			if (L_grid[i][j].type) {
				/* If already allocated, leave it since size doesn't change */
				if (NULL == L_grid[i][j].blocks) {
					L_grid[i][j].blocks = (int *) my_malloc(
							sizeof(int) * L_grid[i][j].type->capacity);

					/* Set them as unconnected */
					for (k = 0; k < L_grid[i][j].type->capacity; ++k) {
						L_grid[i][j].blocks[k] = OPEN;
					}
				}
			}
		}
	}

	/* Go through each block */
	for (i = 0; i < L_num_blocks; ++i) {
		/* Check range of block coords */
		if (block[i].x < 0 || block[i].x > (L_nx + 1) || block[i].y < 0
				|| (block[i].y + block[i].type->height - 1) > (L_ny + 1)
				|| block[i].z < 0 || block[i].z > (block[i].type->capacity)) {
			vpr_printf(TIO_MESSAGE_ERROR,
			"Block %d is at invalid location (%d, %d, %d)\n", i, block[i].x,
					block[i].y, block[i].z);
			exit(1);
		}

		/* Check types match */
		if (block[i].type != L_grid[block[i].x][block[i].y].type) {
			vpr_printf(TIO_MESSAGE_ERROR, "A block is in a grid location "
			"(%d x %d) with a conflicting type.\n", block[i].x, block[i].y);
			exit(1);
		}

		/* Check already in use */
		if (OPEN != L_grid[block[i].x][block[i].y].blocks[block[i].z]) {
			vpr_printf(TIO_MESSAGE_ERROR,
			"Location (%d, %d, %d) is used more than once\n", block[i].x,
					block[i].y, block[i].z);
			exit(1);
		}

		if (L_grid[block[i].x][block[i].y].offset != 0) {
			vpr_printf(TIO_MESSAGE_ERROR,
			"Large block not aligned in placment for block %d at (%d, %d, %d)",
					i, block[i].x, block[i].y, block[i].z);
			exit(1);
		}

		/* Set the block */
		for (j = 0; j < block[i].type->height; j++) {
			L_grid[block[i].x][block[i].y + j].blocks[block[i].z] = i;
			L_grid[block[i].x][block[i].y + j].usage++;
			assert(L_grid[block[i].x][block[i].y + j].offset == j);
		}
	}
}

boolean is_opin(int ipin, t_type_ptr type) {

	/* Returns TRUE if this clb pin is an output, FALSE otherwise. */

	int iclass;

	iclass = type->pin_class[ipin];

	if (type->class_inf[iclass].type == DRIVER)
		return (TRUE);
	else
		return (FALSE);
}

void get_class_range_for_block(INP int iblk, OUTP int *class_low,
		OUTP int *class_high) {
	/* Assumes that the placement has been done so each block has a set of pins allocated to it */
	t_type_ptr type;

	type = block[iblk].type;
	assert(type->num_class % type->capacity == 0);
	*class_low = block[iblk].z * (type->num_class / type->capacity);
	*class_high = (block[iblk].z + 1) * (type->num_class / type->capacity) - 1;
}

int get_max_primitives_in_pb_type(t_pb_type *pb_type) {
	int i, j;
	int max_size, temp_size;
	if (pb_type->modes == 0) {
		max_size = 1;
	} else {
		max_size = 0;
		temp_size = 0;
		for (i = 0; i < pb_type->num_modes; i++) {
			for (j = 0; j < pb_type->modes[i].num_pb_type_children; j++) {
				temp_size += pb_type->modes[i].pb_type_children[j].num_pb
						* get_max_primitives_in_pb_type(
								&pb_type->modes[i].pb_type_children[j]);
			}
			if (temp_size > max_size) {
				max_size = temp_size;
			}
		}
	}
	return max_size;
}

/* finds maximum number of nets that can be contained in pb_type, this is bounded by the number of driving pins */
int get_max_nets_in_pb_type(const t_pb_type *pb_type) {
	int i, j;
	int max_nets, temp_nets;
	if (pb_type->modes == 0) {
		max_nets = pb_type->num_output_pins;
	} else {
		max_nets = 0;
		for (i = 0; i < pb_type->num_modes; i++) {
			temp_nets = 0;
			for (j = 0; j < pb_type->modes[i].num_pb_type_children; j++) {
				temp_nets += pb_type->modes[i].pb_type_children[j].num_pb
						* get_max_nets_in_pb_type(
								&pb_type->modes[i].pb_type_children[j]);
			}
			if (temp_nets > max_nets) {
				max_nets = temp_nets;
			}
		}
	}
	if (pb_type->parent_mode == NULL) {
		max_nets += pb_type->num_input_pins + pb_type->num_output_pins
				+ pb_type->num_clock_pins;
	}
	return max_nets;
}

int get_max_depth_of_pb_type(t_pb_type *pb_type) {
	int i, j;
	int max_depth, temp_depth;
	max_depth = pb_type->depth;
	for (i = 0; i < pb_type->num_modes; i++) {
		for (j = 0; j < pb_type->modes[i].num_pb_type_children; j++) {
			temp_depth = get_max_depth_of_pb_type(
					&pb_type->modes[i].pb_type_children[j]);
			if (temp_depth > max_depth) {
				max_depth = temp_depth;
			}
		}
	}
	return max_depth;
}

/**
 * given a primitive type and a logical block, is the mapping legal
 */
boolean primitive_type_feasible(int iblk, const t_pb_type *cur_pb_type) {
	t_model_ports *port;
	int i, j;
	boolean second_pass;

	if (cur_pb_type == NULL) {
		return FALSE;
	}

	/* check if ports are big enough */
	port = logical_block[iblk].model->inputs;
	second_pass = FALSE;
	while (port || !second_pass) {
		/* TODO: This is slow if the number of ports are large, fix if becomes a problem */
		if (!port) {
			second_pass = TRUE;
			port = logical_block[iblk].model->outputs;
		}
		for (i = 0; i < cur_pb_type->num_ports; i++) {
			if (cur_pb_type->ports[i].model_port == port) {
				for (j = cur_pb_type->ports[i].num_pins; j < port->size; j++) {
					if (port->dir == IN_PORT && !port->is_clock) {
						if (logical_block[iblk].input_nets[port->index][j]
								!= OPEN) {
							return FALSE;
						}
					} else if (port->dir == OUT_PORT) {
						if (logical_block[iblk].output_nets[port->index][j]
								!= OPEN) {
							return FALSE;
						}
					} else {
						assert(port->dir == IN_PORT && port->is_clock);
						assert(j == 0);
						if (logical_block[iblk].clock_net != OPEN) {
							return FALSE;
						}
					}
				}
				break;
			}
		}
		if (i == cur_pb_type->num_ports) {
			if ((logical_block[iblk].model->inputs != NULL && !second_pass)
					|| (logical_block[iblk].model->outputs != NULL
							&& second_pass)) {
				/* physical port not found */
				return FALSE;
			}
		}
		if (port) {
			port = port->next;
		}
	}
	return TRUE;
}


/**
 * Return pb_graph_node pin from model port and pin
 *  NULL if not found
 */
t_pb_graph_pin* get_pb_graph_node_pin_from_model_port_pin(t_model_ports *model_port, int model_pin, t_pb_graph_node *pb_graph_node) {
	int i;

	if(model_port->dir == IN_PORT) {
		if(model_port->is_clock == FALSE) {
			for (i = 0; i < pb_graph_node->num_input_ports; i++) {
				if (pb_graph_node->input_pins[i][0].port->model_port == model_port) {
					if(pb_graph_node->num_input_pins[i] > model_pin) {
						return &pb_graph_node->input_pins[i][model_pin];
					} else {
						return NULL;
					}
				}
			}
		} else {
			for (i = 0; i < pb_graph_node->num_clock_ports; i++) {
				if (pb_graph_node->clock_pins[i][0].port->model_port == model_port) {
					if(pb_graph_node->num_clock_pins[i] > model_pin) {
						return &pb_graph_node->clock_pins[i][model_pin];
					} else {
						return NULL;
					}
				}
			}
		}
	} else {
		assert(model_port->dir == OUT_PORT);
		for (i = 0; i < pb_graph_node->num_output_ports; i++) {
			if (pb_graph_node->output_pins[i][0].port->model_port == model_port) {
				if(pb_graph_node->num_output_pins[i] > model_pin) {
					return &pb_graph_node->output_pins[i][model_pin];
				} else {
					return NULL;
				}
			}
		}
	}
	return NULL;
}

t_pb_graph_pin* get_pb_graph_node_pin_from_vpack_net(int inet, int ipin) {
	int ilogical_block;
	t_model_ports *port;

	ilogical_block = vpack_net[inet].node_block[ipin];

	assert(ilogical_block != OPEN);
	if(logical_block[ilogical_block].pb == NULL) {
		/* This net has not been packed yet thus pb_graph_pin does not exist */
		return NULL;
	}

	if(ipin > 0) {
		port = logical_block[ilogical_block].model->inputs;
		if(vpack_net[inet].is_global) {
			while(port != NULL) {
				if(port->is_clock) {
					if(port->index == vpack_net[inet].node_block_port[ipin]) {
						break;
					}
				}
				port = port->next;
			}
		} else {
			while(port != NULL) {
				if(!port->is_clock) {
					if(port->index == vpack_net[inet].node_block_port[ipin]) {
						break;
					}
				}
				port = port->next;
			}
		}
	} else {
		/* This is an output pin */
		port = logical_block[ilogical_block].model->outputs;
		while(port != NULL) {
			if(port->index == vpack_net[inet].node_block_port[ipin]) {
				break;
			}
			port = port->next;
		}
	}

	assert(port != NULL);
	return get_pb_graph_node_pin_from_model_port_pin(port, ipin, logical_block[ilogical_block].pb->pb_graph_node);
}


t_pb_graph_pin* get_pb_graph_node_pin_from_clb_net(int inet, int ipin) {
	int i, iblock, target_pin, count;
	t_pb_graph_node *pb_graph_node;
	const t_pb_type *pb_type;
	
	iblock = clb_net[inet].node_block[ipin];
	pb_graph_node = block[iblock].pb->pb_graph_node;
	pb_type = pb_graph_node->pb_type;

	target_pin = clb_net[inet].node_block_pin[ipin];
	target_pin %= (pb_type->num_input_pins + pb_type->num_output_pins + pb_type->num_clock_pins);

	if(target_pin < pb_type->num_input_pins) {
		count = target_pin;
		for(i = 0; i < pb_graph_node->num_input_ports; i++) {
			if(count - pb_graph_node->num_input_pins[i] >= 0) {
				count -= pb_graph_node->num_input_pins[i];
			} else {
				return &pb_graph_node->input_pins[i][count];
			}
		}
	} else if (target_pin < pb_type->num_input_pins + pb_type->num_output_pins) {
		count = target_pin - pb_type->num_input_pins;
		for(i = 0; i < pb_graph_node->num_output_ports; i++) {
			if(count - pb_graph_node->num_output_pins[i] >= 0) {
				count -= pb_graph_node->num_output_pins[i];
			} else {
				return &pb_graph_node->output_pins[i][count];
			}
		}
	} else {
		count = target_pin - pb_type->num_input_pins - pb_type->num_output_pins;
		for(i = 0; i < pb_graph_node->num_clock_ports; i++) {
			if(count - pb_graph_node->num_clock_pins[i] >= 0) {
				count -= pb_graph_node->num_clock_pins[i];
			} else {
				return &pb_graph_node->clock_pins[i][count];
			}
		}
	}
	assert(0);
	return NULL;
}


/**
 * Determine cost for using primitive within a complex block, should use primitives of low cost before selecting primitives of high cost
 For now, assume primitives that have a lot of pins are scarcer than those without so use primitives with less pins before those with more
 */
float compute_primitive_base_cost(INP t_pb_graph_node *primitive) {
	return (primitive->pb_type->num_input_pins
			+ primitive->pb_type->num_output_pins
			+ primitive->pb_type->num_clock_pins);
}

int num_ext_inputs_logical_block(int iblk) {

	/* Returns the number of input pins on this logical_block that must be hooked *
	 * up through external interconnect.  That is, the number of input    *
	 * pins used - the number which connect (internally) to the outputs.   */

	int ext_inps, output_net, ipin, opin;

	t_model_ports *port, *out_port;

	/* TODO: process to get ext_inps is slow, should cache in lookup table */
	ext_inps = 0;
	port = logical_block[iblk].model->inputs;
	while (port) {
		if (port->is_clock == FALSE) {
			for (ipin = 0; ipin < port->size; ipin++) {
				if (logical_block[iblk].input_nets[port->index][ipin] != OPEN) {
					ext_inps++;
				}
				out_port = logical_block[iblk].model->outputs;
				while (out_port) {
					for (opin = 0; opin < out_port->size; opin++) {
						output_net =
								logical_block[iblk].output_nets[out_port->index][opin];
						if (output_net == OPEN)
							continue;
						/* TODO: I could speed things up a bit by computing the number of inputs *
						 * and number of external inputs for each logic logical_block at the start of   *
						 * clustering and storing them in arrays.  Look into if speed is a      *
						 * problem.                                                             */

						if (logical_block[iblk].input_nets[port->index][ipin]
								== output_net) {
							ext_inps--;
							break;
						}
					}
					out_port = out_port->next;
				}
			}
		}
		port = port->next;
	}

	assert(ext_inps >= 0);

	return (ext_inps);
}


void free_cb(t_pb *pb) {
	const t_pb_type * pb_type;
	int i, total_nodes;

	pb_type = pb->pb_graph_node->pb_type;

	total_nodes = pb->pb_graph_node->total_pb_pins + pb_type->num_input_pins
			+ pb_type->num_output_pins + pb_type->num_clock_pins;

	for (i = 0; i < total_nodes; i++) {
		if (pb->rr_graph[i].edges != NULL) {
			free(pb->rr_graph[i].edges);
		}
		if (pb->rr_graph[i].switches != NULL) {
			free(pb->rr_graph[i].switches);
		}
	}
	free(pb->rr_graph);
	free_pb(pb);
}

void free_pb(t_pb *pb) {
	const t_pb_type * pb_type;
	int i, j, mode;
	struct s_linked_vptr *revalid_molecule;
	t_pack_molecule *cur_molecule;

	pb_type = pb->pb_graph_node->pb_type;

	if (pb_type->blif_model == NULL) {
		mode = pb->mode;
		for (i = 0;
				i < pb_type->modes[mode].num_pb_type_children
						&& pb->child_pbs != NULL; i++) {
			for (j = 0;
					j < pb_type->modes[mode].pb_type_children[i].num_pb
							&& pb->child_pbs[i] != NULL; j++) {
				if (pb->child_pbs[i][j].name != NULL || pb->child_pbs[i][j].child_pbs != NULL) {
					free_pb(&pb->child_pbs[i][j]);
				}
			}
			if (pb->child_pbs[i])
				free(pb->child_pbs[i]);
		}
		if (pb->child_pbs)
			free(pb->child_pbs);
		pb->child_pbs = NULL;

		if (pb->local_nets != NULL) {
			for (i = 0; i < pb->num_local_nets; i++) {
				free(pb->local_nets[i].node_block);
				free(pb->local_nets[i].node_block_port);
				free(pb->local_nets[i].node_block_pin);
				if (pb->local_nets[i].name != NULL) {
					free(pb->local_nets[i].name);
				}
			}
			free(pb->local_nets);
			pb->local_nets = NULL;
		}

		if (pb->rr_node_to_pb_mapping != NULL) {
			free(pb->rr_node_to_pb_mapping);
			pb->rr_node_to_pb_mapping = NULL;
		}
		
		if (pb->name)
			free(pb->name);
		pb->name = NULL;
	} else {
		/* Primitive */
		if (pb->name)
			free(pb->name);
		pb->name = NULL;
		if (pb->lut_pin_remap) {
			free(pb->lut_pin_remap);
		}
		pb->lut_pin_remap = NULL;
		if (pb->logical_block != OPEN && logical_block != NULL) {
			logical_block[pb->logical_block].clb_index = NO_CLUSTER;
			logical_block[pb->logical_block].pb = NULL;
			/* If any molecules were marked invalid because of this logic block getting packed, mark them valid */
			revalid_molecule = logical_block[pb->logical_block].packed_molecules;
			while (revalid_molecule != NULL) {
				cur_molecule = (t_pack_molecule*)revalid_molecule->data_vptr;
				if (cur_molecule->valid == FALSE) {
					for (i = 0; i < get_array_size_of_molecule(cur_molecule); i++) {
						if (cur_molecule->logical_block_ptrs[i] != NULL) {
							if (cur_molecule->logical_block_ptrs[i]->clb_index != OPEN) {
								break;
							}
						}
					}
					/* All logical blocks are open for this molecule, place back in queue */
					if (i == get_array_size_of_molecule(cur_molecule)) {
						cur_molecule->valid = TRUE;	
					}
				}
				revalid_molecule = revalid_molecule->next;
			}
		}
		pb->logical_block = OPEN;
	}
	free_pb_stats(pb);
	pb->pb_stats.gain = NULL;
}

void free_pb_stats(t_pb *pb) {
	int i;
	t_pb_graph_node *pb_graph_node = pb->pb_graph_node;

	if (pb->pb_stats.gain != NULL) {
		free(pb->pb_stats.gain);
		free(pb->pb_stats.timinggain);
		free(pb->pb_stats.sharinggain);
		free(pb->pb_stats.hillgain);
		free(pb->pb_stats.connectiongain);
		for (i = 0; i < pb_graph_node->num_input_pin_class; i++) {
			free(pb->pb_stats.input_pins_used[i]);
			free(pb->pb_stats.lookahead_input_pins_used[i]);
		}
		free(pb->pb_stats.input_pins_used);
		free(pb->pb_stats.lookahead_input_pins_used);
		for (i = 0; i < pb_graph_node->num_output_pin_class; i++) {
			free(pb->pb_stats.output_pins_used[i]);
			free(pb->pb_stats.lookahead_output_pins_used[i]);
		}
		free(pb->pb_stats.output_pins_used);
		free(pb->pb_stats.lookahead_output_pins_used);
		free(pb->pb_stats.feasible_blocks);
		free(pb->pb_stats.num_pins_of_net_in_pb);
		free(pb->pb_stats.marked_nets);
		free(pb->pb_stats.marked_blocks);
		pb->pb_stats.gain = NULL;
	}
}

int ** alloc_and_load_net_pin_index() {
	/* Allocates and loads net_pin_index array, this array allows us to quickly   *
	 * find what pin on the net a block pin corresponds to. Returns the pointer   *
	 * to the 2D net_pin_index array.                                             */

	int inet, netpin, blk, iblk, ipin, itype, **temp_net_pin_index, max_pins_per_clb = 0;
	t_type_ptr type;

	/* Compute required size. */
	for (itype = 0; itype < num_types; itype++)
		max_pins_per_clb = max(max_pins_per_clb, type_descriptors[itype].num_pins);
	
	/* Allocate for maximum size. */
	temp_net_pin_index = (int **) alloc_matrix(0, num_blocks - 1, 0,
				max_pins_per_clb - 1, sizeof(int));

	/* Initialize values to OPEN */
	for (iblk = 0; iblk < num_blocks; iblk++) {
		type = block[iblk].type;
		for (ipin = 0; ipin < type->num_pins; ipin++) {
			temp_net_pin_index[iblk][ipin] = OPEN;
		}
	}

	/* Load the values */
	for (inet = 0; inet < num_nets; inet++) {
		if (clb_net[inet].is_global)
			continue;
		for (netpin = 0; netpin <= clb_net[inet].num_sinks; netpin++) {
			blk = clb_net[inet].node_block[netpin];
			temp_net_pin_index[blk][clb_net[inet].node_block_pin[netpin]] = netpin;
		}
	}

	/* Returns the pointers to the 2D array. */
	return temp_net_pin_index;
}

void alloc_and_load_blk_pin_to_port_pin(int *** blk_pin_to_port, int *** blk_pin_to_port_pin) {
	
	/* Allocates and loads blk_pin_to_port and blk_pin_to_port_pin arrays.   *
	
	 * blk_pin_to_port array allow us to quickly find what port a block pin  *
	 * corresponds to.                                                       *
	 * [0...num_blocks-1][0...max_pins_per_clb-1]                            *

	 * blk_pin_to_port_pin array allow us to quickly find what port pin a    *
	 * block pin corresponds to.                                             *
	 * [0...num_blocks-1][0...max_pins_per_clb-1]                            *

	 * Stores the pointers to the two 2D arrays in the addresses passed in.  */

	int ** temp_blk_pin_to_port = NULL;
	int ** temp_blk_pin_to_port_pin = NULL;
	int itype, iblk, iblk_pin, iport, iport_pin;
	int blk_pin_count, num_port_pins, num_ports, max_pins_per_clb;

	/* Compute required size. */
	max_pins_per_clb = 0;
	for (itype = 0; itype < num_types; itype++)
		max_pins_per_clb = max(max_pins_per_clb, type_descriptors[itype].num_pins);
	
	/* Allocate for maximum size. */
	temp_blk_pin_to_port = (int **) alloc_matrix(0, num_blocks - 1, 0,
				max_pins_per_clb - 1, sizeof(int));
	temp_blk_pin_to_port_pin = (int **) alloc_matrix(0, num_blocks - 1, 0,
				max_pins_per_clb - 1, sizeof(int));

	/* Initialize values to OPEN */
	for (iblk = 0; iblk < num_blocks; iblk++) {
		for (iblk_pin = 0; iblk_pin < max_pins_per_clb; iblk_pin++) {
			temp_blk_pin_to_port[iblk][iblk_pin] = OPEN;
			temp_blk_pin_to_port_pin[iblk][iblk_pin] = OPEN;
		}
	}

	/* Load the values */
	for (iblk = 0; iblk < num_blocks; iblk++) {
		blk_pin_count = 0;
		num_ports = block[iblk].type->pb_type->num_ports;
		for (iport = 0; iport < num_ports; iport++) {
			num_port_pins = block[iblk].type->pb_type->ports[iport].num_pins;
			for (iport_pin = 0; iport_pin < num_port_pins; iport_pin++) {
				temp_blk_pin_to_port[iblk][blk_pin_count] = iport;
				temp_blk_pin_to_port_pin[iblk][blk_pin_count] = iport_pin;
				blk_pin_count++;
			}
		}
	}

	/* Returns the pointers to the 2D arrays. */
	*blk_pin_to_port = temp_blk_pin_to_port;
	*blk_pin_to_port_pin = temp_blk_pin_to_port_pin;
}

int *** alloc_and_load_port_pin_to_blk_pin(void) {

	/* Allocates and loads port_pin_to_block_pin array.                      *

	 * port_pin_to_block_pin array allows us to quickly find what block pin  *
	 * a port pin corresponds to.                                            *
	 * [0...num_blocks-1][0...max_ports_per_blk-1][0...max_pins_per_port-1]  *

	 * Returns the pointer to the port_pin_to_block_pin 3D array.            */

	int *** temp_port_pin_to_blk_pin = NULL;
	int itype, iblk, iport, iport_pin;
	int blk_pin_count, num_port_pins, num_ports;
	int max_ports_per_blk, max_pins_per_port;

	/* Compute required size. */
	max_ports_per_blk = 0;
	max_pins_per_port = 0;
	for (itype = 1; itype < num_types; itype++) {
		max_ports_per_blk = max(max_ports_per_blk, type_descriptors[itype].pb_type->num_ports);
		for (iport = 0; iport < type_descriptors[itype].pb_type->num_ports; iport++) {
			max_pins_per_port = max(max_pins_per_port, type_descriptors[itype].pb_type->ports->num_pins);
		}
	}
	
	/* Allocate for maximum size. */
	temp_port_pin_to_blk_pin = (int ***) my_malloc(num_blocks * sizeof(int**));
	for (iblk = 0; iblk < num_blocks; iblk++) {
		temp_port_pin_to_blk_pin[iblk] = (int **) my_malloc(max_ports_per_blk * sizeof(int*));
		for (iport = 0; iport < max_ports_per_blk; iport++) {
			temp_port_pin_to_blk_pin[iblk][iport] = (int *) my_malloc(max_pins_per_port * sizeof(int));
		}
	}
	
	/* Initialize values to OPEN */
	for (iblk = 0; iblk < num_blocks; iblk++) {
		for (iport = 0; iport < max_ports_per_blk; iport++) {
			for (iport_pin = 0; iport_pin < max_pins_per_port; iport_pin++) {
				temp_port_pin_to_blk_pin[iblk][iport][iport_pin] = OPEN;
			}
		}
	}

	/* Load the values */
	for (iblk = 0; iblk < num_blocks; iblk++) {
		blk_pin_count = 0;
		num_ports = block[iblk].type->pb_type->num_ports;
		for (iport = 0; iport < num_ports; iport++) {
			num_port_pins = block[iblk].type->pb_type->ports[iport].num_pins;
			for (iport_pin = 0; iport_pin < num_port_pins; iport_pin++) {
				temp_port_pin_to_blk_pin[iblk][iport][iport_pin] = blk_pin_count;
				blk_pin_count++;
			}
		}
	}
	
	/* Returns the pointer to the 3D array. */
	return temp_port_pin_to_blk_pin;
}

void alloc_and_load_blk_pin_to_idirect(t_direct_inf* directs, int num_directs, 
		int *** port_pin_to_blk_pin, 
		int *** blk_pin_to_idirect, int *** blk_pin_to_direct_src_or_sink) {

	/* Allocates and loads blk_pin_to_idirect and blk_pin_to_direct_src_or_sink arrays. *

	 * blk_pin_to_idirect array allow us to quickly find pins that could be in a direct *
	 * connection. Values stored is the index of the possible direct connection as      *
	 * specified in the arch file, OPEN (-1) is stored for pins that could not be part  *
	 * of a direct chain conneciton.                                                    *

	 * blk_pin_to_direct_src_or_sink array stores the value SOURCE if the pin is the    *
	 * from_pin, SINK if the pin is the to_pin in the direct connection as specified in *
	 * the arch file, OPEN (-1) is stored for pins that could not be part of a direct   *
	 * chain conneciton.                                                                *
	 
	 * Stores the pointers to the two 2D arrays in the addresses passed in.             */

	int itype, max_pins_per_clb;
	int ** temp_blk_pin_to_idirect, ** temp_blk_pin_to_direct_src_or_sink;

	int iblk, iblk_pin, iport, iport_pin, idirect, ichar;
	int num_ports, num_port_pins;

	char to_pb_type_name[MAX_STRING_LEN], to_port_name[MAX_STRING_LEN], 
			from_pb_type_name[MAX_STRING_LEN], from_port_name[MAX_STRING_LEN], 
			source_string[MAX_STRING_LEN];
	char * find_format = NULL;
	int to_start_pin_index = -1, to_end_pin_index = -1;
	int from_start_pin_index = -1, from_end_pin_index = -1;
	int match_count;
		
	/* Compute required size. */
	max_pins_per_clb = 0;
	for (itype = 0; itype < num_types; itype++)
		max_pins_per_clb = max(max_pins_per_clb, type_descriptors[itype].num_pins);
	
	/* Allocate for maximum size. */
	temp_blk_pin_to_idirect = (int **) alloc_matrix(0, num_blocks - 1, 0,
				max_pins_per_clb - 1, sizeof(int));
	temp_blk_pin_to_direct_src_or_sink = (int **) alloc_matrix(0, num_blocks - 1, 0,
				max_pins_per_clb - 1, sizeof(int));
	
	/* Initialize values to OPEN */
	for (iblk = 0; iblk < num_blocks; iblk++) {
		for (iblk_pin = 0; iblk_pin < max_pins_per_clb; iblk_pin++) {
			temp_blk_pin_to_idirect[iblk][iblk_pin] = OPEN;
			temp_blk_pin_to_direct_src_or_sink[iblk][iblk_pin] = OPEN;
		}
	}

	/* Load the values */
	// Go through directs and find pins with possible direct connections
	for (idirect = 0; idirect < num_directs; idirect++) {
		// parse out the pb_type and port name, possibly pin_indices from from_pin
		find_format = strstr(directs[idirect].from_pin,"[");
		if (find_format == NULL) {
			/* Format "pb_type_name.port_name" */
			from_start_pin_index = from_end_pin_index = -1;
			
			strcpy (source_string, directs[idirect].from_pin);
			for (ichar = 0; ichar < (int)(strlen(source_string)); ichar++) {
				if (source_string[ichar] == '.')
					source_string[ichar] = ' ';
			}

			match_count = sscanf(source_string, "%s %s", from_pb_type_name, from_port_name);
			if (match_count != 2){
				vpr_printf(TIO_MESSAGE_ERROR, "[LINE %d] Invalid from_pin - %s, "
						"name should be in the format \"pb_type_name\".\"port_name\" or "
						"\"pb_type_name\".\"port_name [end_pin_index:start_pin_index]\". "
						" The end_pin_index and start_pin_index can be the same.\n", directs[idirect].line,
						directs[idirect].from_pin);
				exit(1);
			}
		} else {
			/* Format "pb_type_name.port_name [end_pin_index:start_pin_index]" */
			strcpy (source_string, directs[idirect].from_pin);
			for (ichar = 0; ichar < (int)(strlen(source_string)); ichar++) {
				if (source_string[ichar] == '.')
					source_string[ichar] = ' ';
			}

			match_count = sscanf(source_string, "%s %s [%d:%d]", 
									from_pb_type_name, from_port_name, 
									&from_end_pin_index, &from_start_pin_index);
			if (match_count != 4){
				vpr_printf(TIO_MESSAGE_ERROR, "[LINE %d] Invalid from_pin - %s, "
						"name should be in the format \"pb_type_name\".\"port_name\" or "
						"\"pb_type_name\".\"port_name [end_pin_index:start_pin_index]\". "
						" The end_pin_index and start_pin_index can be the same.\n", directs[idirect].line,
						directs[idirect].from_pin);
				exit(1);
			}
			if (from_end_pin_index < 0 || from_start_pin_index < 0) {
				vpr_printf(TIO_MESSAGE_ERROR, "[LINE %d] Invalid from_pin - %s, "
						"the pin_index [end_pin_index:start_pin_index] should not "
						"be a negative value.\n", directs[idirect].line, directs[idirect].from_pin);
				exit(1);
			}
			if ( from_end_pin_index < from_start_pin_index) {
				vpr_printf(TIO_MESSAGE_ERROR, "[LINE %d] Invalid from_pin - %s, "
						"the end_pin_index in [end_pin_index:start_pin_index] should "
						"not be less than start_pin_index.\n", directs[idirect].line, directs[idirect].from_pin);
				exit(1);
			}
		}
			
		// parse out the pb_type and port name, possibly pin_indices from to_pin
		find_format = strstr(directs[idirect].to_pin,"[");
		if (find_format == NULL) {
			/* Format "pb_type_name.port_name" */
			to_start_pin_index = to_end_pin_index = -1;
			
			strcpy (source_string, directs[idirect].to_pin);
			for (ichar = 0; ichar < (int)(strlen(source_string)); ichar++) {
				if (source_string[ichar] == '.')
					source_string[ichar] = ' ';
			}

			match_count = sscanf(source_string, "%s %s", to_pb_type_name, to_port_name);
			if (match_count != 2){
				vpr_printf(TIO_MESSAGE_ERROR, "[LINE %d] Invalid to_pin - %s, "
						"name should be in the format \"pb_type_name\".\"port_name\" or "
						"\"pb_type_name\".\"port_name [end_pin_index:start_pin_index]\". "
						" The end_pin_index and start_pin_index can be the same.\n", directs[idirect].line,
						directs[idirect].to_pin);
				exit(1);
			}
		} else {
			/* Format "pb_type_name.port_name [end_pin_index:start_pin_index]" */
			strcpy (source_string, directs[idirect].to_pin);
			for (ichar = 0; ichar < (int)(strlen(source_string)); ichar++) {
				if (source_string[ichar] == '.')
					source_string[ichar] = ' ';
			}

			match_count = sscanf(source_string, "%s %s [%d:%d]", 
									to_pb_type_name, to_port_name, 
									&to_end_pin_index, &to_start_pin_index);
			if (match_count != 4){
				vpr_printf(TIO_MESSAGE_ERROR, "[LINE %d] Invalid to_pin - %s, "
						"name should be in the format \"pb_type_name\".\"port_name\" or "
						"\"pb_type_name\".\"port_name [end_pin_index:start_pin_index]\". "
						" The end_pin_index and start_pin_index can be the same.\n", directs[idirect].line,
						directs[idirect].to_pin);
				exit(1);
			}
			if (to_end_pin_index < 0 || to_start_pin_index < 0) {
				vpr_printf(TIO_MESSAGE_ERROR, "[LINE %d] Invalid to_pin - %s, "
						"the pin_index [end_pin_index:start_pin_index] should not "
						"be a negative value.\n", directs[idirect].line, directs[idirect].to_pin);
				exit(1);
			}
			if (to_end_pin_index < to_start_pin_index) {
				vpr_printf(TIO_MESSAGE_ERROR, "[LINE %d] Invalid to_pin in - %s, "
						"the end_pin_index in [end_pin_index:start_pin_index] should "
						"not be less than start_pin_index.\n", directs[idirect].line, directs[idirect].to_pin);
				exit(1);
			}
		}
		
		// Now I have all the data that I need, I could find mark down blocks having possible connections
		for (iblk = 0; iblk < num_blocks; iblk++) {
			// Find blocks with the same name as from_pb_type_name
			if (strcmp(block[iblk].type->pb_type->name, from_pb_type_name) == 0) {
				
				num_ports = block[iblk].type->pb_type->num_ports;
				for (iport = 0; iport < num_ports; iport++) {
					// Find ports with the same name as from_port_name
					if (strcmp(block[iblk].type->pb_type->ports[iport].name, from_port_name) == 0) {
						
						num_port_pins = block[iblk].type->pb_type->ports[iport].num_pins;

						// Check whether the end_pin_index is valid
						if (from_end_pin_index > num_port_pins) {
							vpr_printf(TIO_MESSAGE_ERROR, "[LINE %d] Invalid from_pin - %s, "
									"the end_pin_index in [end_pin_index:start_pin_index] should "
									"be less than the num_port_pins %d.\n", directs[idirect].line,
									directs[idirect].from_pin, num_port_pins);
							exit(1);
						}

						// Check whether the pin indices are specified
						if (from_start_pin_index >= 0 || from_end_pin_index >= 0) {

							// Mark pins with indices from from_start_pin_index to from_end_pin_index, inclusive
							for (iport_pin = from_start_pin_index; iport_pin <= from_end_pin_index; iport_pin++) {

								iblk_pin = port_pin_to_blk_pin[iblk][iport][iport_pin];

								// Check the fc for the pin, direct chain link only if fc == 0
								if (block[iblk].type->Fc[iblk_pin] == 0) {
								
									temp_blk_pin_to_idirect[iblk][iblk_pin] = idirect;
							
									// Check whether the pins are marked, errors out if so
									if (temp_blk_pin_to_direct_src_or_sink[iblk][iblk_pin] != OPEN) {
										vpr_printf(TIO_MESSAGE_ERROR, "[LINE %d] Invalid from_pin - %s and to_pin - %s, "
											"there are pins overlap.\n", directs[idirect].line, 
											directs[idirect].from_pin, directs[idirect].to_pin);
										exit(1);
									} else {
										temp_blk_pin_to_direct_src_or_sink[iblk][iblk_pin] = SOURCE;
									}

								}
							} // Finish marking all the pins

						} else {

							// Mark all the pins in this port
							for (iport_pin = 0; iport_pin < num_port_pins; iport_pin++) {

								iblk_pin = port_pin_to_blk_pin[iblk][iport][iport_pin];

								// Check the fc for the pin, direct chain link only if fc == 0
								if (block[iblk].type->Fc[iblk_pin] == 0) {
								
									temp_blk_pin_to_idirect[iblk][iblk_pin] = idirect;
							
									// Check whether the pins are marked, errors out if so
									if (temp_blk_pin_to_direct_src_or_sink[iblk][iblk_pin] != OPEN) {
										vpr_printf(TIO_MESSAGE_ERROR, "[LINE %d] Invalid from_pin - %s and to_pin - %s, "
											"there are pins overlap.\n", directs[idirect].line,
											directs[idirect].from_pin, directs[idirect].to_pin);
										exit(1);
									} else {
										temp_blk_pin_to_direct_src_or_sink[iblk][iblk_pin] = SOURCE;
									}

								}
							} // Finish marking all the pins

						}
					} // Do nothing if port_name does not match
				} // Finish going through all the ports
			} // Do nothing if pb_type_name does not match

			// Then, find blocks with the same name as to_pb_type_name
			// Note: not using else/elseif here since the pb_type could be the 
			// same for to and to pb_type
			if (strcmp(block[iblk].type->pb_type->name, to_pb_type_name) == 0) {

				num_ports = block[iblk].type->pb_type->num_ports;
				for (iport = 0; iport < num_ports; iport++) {
					// Find ports with the same name as to_port_name
					if (strcmp(block[iblk].type->pb_type->ports[iport].name, to_port_name) == 0) {
						
						num_port_pins = block[iblk].type->pb_type->ports[iport].num_pins;

						// Check whether the end_pin_index is valid
						if (to_end_pin_index > num_port_pins) {
							vpr_printf(TIO_MESSAGE_ERROR, "[LINE %d] Invalid to_pin - %s, "
									"the end_pin_index in [end_pin_index:start_pin_index] should "
									"be less than the num_port_pins %d.\n", directs[idirect].line,
									directs[idirect].to_pin, num_port_pins);
							exit(1);
						}
						
						// Check whether the pin indices are specified
						if (from_start_pin_index >= 0 || from_end_pin_index >= 0) {

							// Mark pins with indices from to_start_pin_index to to_end_pin_index, inclusive
							for (iport_pin = to_start_pin_index; iport_pin <= to_end_pin_index; iport_pin++) {

								iblk_pin = port_pin_to_blk_pin[iblk][iport][iport_pin];
							
								// Check the fc for the pin, direct chain link only if fc == 0
								if (block[iblk].type->Fc[iblk_pin] == 0) {
								
									temp_blk_pin_to_idirect[iblk][iblk_pin] = idirect;
								
									// Check whether the pins are marked, errors out if so
									if (temp_blk_pin_to_direct_src_or_sink[iblk][iblk_pin] != OPEN) {
										vpr_printf(TIO_MESSAGE_ERROR, "[LINE %d] Invalid from_pin - %s and to_pin - %s, "
											"there are pins overlap.\n", directs[idirect].line, 
											directs[idirect].from_pin, directs[idirect].to_pin);
										exit(1);
									} else {
										temp_blk_pin_to_direct_src_or_sink[iblk][iblk_pin] = SINK;
									}

									// Only support chains with block height 1
									assert(block[iblk].type->height == 1);
								}
							} // Finish marking all the pins

						} else {

							// Mark all the pins in this port
							for (iport_pin = 0; iport_pin < num_port_pins; iport_pin++) {

								iblk_pin = port_pin_to_blk_pin[iblk][iport][iport_pin];
							
								// Check the fc for the pin, direct chain link only if fc == 0
								if (block[iblk].type->Fc[iblk_pin] == 0) {
								
									temp_blk_pin_to_idirect[iblk][iblk_pin] = idirect;
								
									// Check whether the pins are marked, errors out if so
									if (temp_blk_pin_to_direct_src_or_sink[iblk][iblk_pin] != OPEN) {
										vpr_printf(TIO_MESSAGE_ERROR, "[LINE %d] Invalid from_pin - %s and to_pin - %s, "
											"there are pins overlap.\n", directs[idirect].line,
											directs[idirect].from_pin, directs[idirect].to_pin);
										exit(1);
									} else {
										temp_blk_pin_to_direct_src_or_sink[iblk][iblk_pin] = SINK;
									}
									
									// Only support chains with block height 1
									assert(block[iblk].type->height == 1);
								}
							} // Finish marking all the pins

						}
					} // Do nothing if port_name does not match
				} // Finish going through all the ports
			} // Do nothing if pb_type_name does not match

		} // Finish going through all the blocks
	} // Finish going through all the directs


	/* Returns the pointer to the 2D arrays. */
	*blk_pin_to_idirect = temp_blk_pin_to_idirect;
	*blk_pin_to_direct_src_or_sink = temp_blk_pin_to_direct_src_or_sink;

}
