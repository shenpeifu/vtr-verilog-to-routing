//#if 0
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "util.h"
#include "vpr_types.h"
#include "physical_types.h"
#include "globals.h"
#include "mst.h"
#include "place.h"
#include "read_xml_arch_file.h"
#include "ReadOptions.h"
#include "place_macro.h"
#include "vpr_utils.h"


/* These are the file scope variables declarations. */
static int ** blk_pin_to_port = NULL;
static int ** blk_pin_to_port_pin = NULL;
static int *** port_pin_to_blk_pin = NULL;
static int ** blk_pin_to_idirect = NULL;
static int ** blk_pin_to_direct_src_or_sink = NULL;

/* These are the file scope subroutines declarations. */
static t_pl_macro * alloc_and_load_placement_macros (t_direct_inf* directs, int num_directs);

/* These are the file scope subroutines defintions. */
t_pl_macro * init_alloc_and_load_placement_macros(t_direct_inf* directs, int num_directs){
	
	/* This function sets up the necessary file scope variables needed before      *
	 * invoking the actual alloc_and_load_placement_macros function.               */

	t_pl_macro * chain = NULL;
	
	/* Sets up the required variables. */
	port_pin_to_blk_pin = alloc_and_load_port_pin_to_blk_pin();
	alloc_and_load_blk_pin_to_port_pin(&blk_pin_to_port, &blk_pin_to_port_pin);
		
	alloc_and_load_blk_pin_to_idirect(directs, num_directs, port_pin_to_blk_pin,
			&blk_pin_to_idirect, &blk_pin_to_direct_src_or_sink);

	/* Invokes the actial alloc_and_load_placement_macros function. */
	chain = alloc_and_load_placement_macros (directs, num_directs);

	/* Frees up the temporary data structures. */
	/*
	free_matrix(blk_pin_to_port);
	free_matrix(blk_pin_to_port_pin);
	free_matrix(port_pin_to_blk_pin);
	free_matrix(blk_pin_to_idirect);
	free_matrix(blk_pin_to_direct_src_or_sink);
	*/

	/* Returns the pointer to the chain. */
	return (chain);

}

static t_pl_macro * alloc_and_load_placement_macros (t_direct_inf* directs, int num_directs) {

	int iblk, from_iblk_pin, to_iblk_pin, from_inet, to_inet, from_idirect, to_idirect, from_src_or_sink, to_src_or_sink;
	int num_blk_pins, num_pl_macro_head, 
	int *pl_macro_idirect, *pl_macro_num_members, **pl_macro_member_blk_num;
	int next_iblk, next_iblk_pin, next_inet, next_src_or_sink;
	int ichain, imember;
	t_pl_macro * temp_chain = NULL;

	num_pl_macro_head = 0;
	pl_macro_num_members = (int *) my_calloc (num_blocks , sizeof(int));
	pl_macro_idirect = (int *) my_calloc (num_blocks , sizeof(int));
	pl_macro_member_blk_num = (int **) my_calloc (num_blocks , sizeof(int*));
	for (iblk = 0; iblk < num_blocks; iblk++) 
		pl_macro_member_blk_num[iblk] = (int *) my_calloc (num_blocks , sizeof(int));

	/* Compute required size. */
	// Go through all the pins with possible direct connections in blk_pin_to_idirect
	// Count the number of heads (which is the same as the number chains) and also the length of each chain
	// Head - blocks with from_pin OPEN and to_pin connected
	// Tail - blocks with from_pin connected and to_pin OPEN
	for (iblk = 0; iblk < num_blocks; iblk++) {

		num_blk_pins = block[iblk].type->num_pins;
		for (from_iblk_pin = 0; from_iblk_pin < num_blk_pins; from_iblk_pin++) {
			
			from_inet = block[iblk].nets[from_iblk_pin];
			from_idirect = blk_pin_to_idirect[iblk][from_iblk_pin];
			from_src_or_sink = blk_pin_to_direct_src_or_sink[iblk][from_iblk_pin];
			
			// Find from_pins with possible direct connection, make sure that it is not connected to any net
			if ( from_src_or_sink == SOURCE && from_idirect != OPEN && from_inet == OPEN ) {

				for (to_iblk_pin = 0; to_iblk_pin < num_blk_pins; to_iblk_pin++) {
					to_inet = block[iblk].nets[to_iblk_pin];
					to_idirect = blk_pin_to_idirect[iblk][to_iblk_pin];
					to_src_or_sink = blk_pin_to_direct_src_or_sink[iblk][to_iblk_pin];

					// Find to_pins with the same possible direct connection.
					if ( to_src_or_sink == SINK && to_idirect == from_idirect && to_inet != OPEN) {
						
						// Mark down the first block in the chain
						pl_macro_member_blk_num[num_pl_macro_head][0] = iblk;
						pl_macro_idirect[num_pl_macro_head] = to_idirect;
						
						// Also find out how many members are in the macros, there are at least 2 members - 1 head and 1 tail.
						next_inet = to_inet;
						do {
						
							// Make sure that it is connected to a net and only has 1 sink - direct chain connection.
							assert(clb_net[next_inet].num_sinks == 1);
							
							// Increment the num_member count.
							pl_macro_num_members[num_pl_macro_head]++;
							
							// Find the next block, make sure that they are consistent :-
							// 1) from the coord offsets specified in the arch file.
							// 2) from the netlist specified in the blif file.

							next_iblk = clb_net[next_inet].node_block[1];
							next_iblk_pin = clb_net[next_inet].node_block_pin[1];
							
							// Assert that the block connected next is indeed the block at the right offset.
							assert ( next_iblk == *(grid[block[next_iblk].x + directs[to_idirect].x_offset]
														[block[next_iblk].y + directs[to_idirect].y_offset].blocks));
							
							// Find the following block connected.
							next_src_or_sink = blk_pin_to_direct_src_or_sink[next_iblk][next_iblk_pin];
							next_inet = block[next_iblk].nets[next_iblk_pin];
							
							// Mark down the next blk in the chain
							pl_macro_member_blk_num[num_pl_macro_head][pl_macro_num_members[num_pl_macro_head]] = next_iblk;

						} while(next_src_or_sink != OPEN);

						num_pl_macro_head ++;

					} // Do nothing if the to_pins does not have same possible direct connection.
				} // Finish going through all the pins for to_pins.
			} // Do nothing if the from_pins does not have same possible direct connection.
		} // Finish going through all the pins for from_pins.
	} // Finish going through all blocks.
	// Now, all the data is readily stored in the temporary data structures.


	/* Allocate the memories for the chain. */
	temp_chain = (t_pl_macro *) my_malloc (num_pl_macro_head * sizeof(t_pl_macro));

	/* Allocate the memories for the chaim members.             *
	 * Load the values from the temporary data structures.      */
	for (ichain = 0; ichain < num_pl_macro_head; ichain++) {
		temp_chain[ichain].num_blocks = pl_macro_num_members[ichain];
		temp_chain[ichain].members = (t_pl_macro_member *) my_malloc 
										(temp_chain[ichain].num_blocks * sizeof(t_pl_macro_member));
		temp_chain[ichain].idirect = pl_macro_idirect[ichain];

		/* Load the values for each member of the chain */
		for (imember = 0; imember < temp_chain[ichain].num_blocks; ) {
			temp_chain[ichain].members[imember].x_offset = directs[temp_chain[ichain].idirect].x_offset;
			temp_chain[ichain].members[imember].y_offset = directs[temp_chain[ichain].idirect].y_offset;
			temp_chain[ichain].members[imember].blk_index = pl_macro_member_blk_num[ichain][imember];
		}
	}

	/* Frees up the temporary data structures. */
	free(pl_macro_num_members);
	free(pl_macro_idirect);
	free(pl_macro_member_blk_num);

	/* Returns the pointer to the chain array. */
	return (temp_chain);
}


//#endif