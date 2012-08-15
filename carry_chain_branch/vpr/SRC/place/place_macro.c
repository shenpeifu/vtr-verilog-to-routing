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


/*** These are the file scope variables declarations. ***/

/* blk_pin_to_port maps a blk_num and blk_pin_num to the corresponding   *
 * port_num.                                                             *
 * [0...num_blocks-1][0...max_pins_per_clb-1]                            */
static int ** blk_pin_to_port = NULL;

/* blk_pin_to_port_pin maps a blk_num and blk_pin_num to the             *
 * corresponding port_pin_num.                                           *
 * [0...num_blocks-1][0...max_pins_per_clb-1]                            */
static int ** blk_pin_to_port_pin = NULL;

/* port_pin_to_blk_pin maps a blk_num port_num and port_pin_num to the   *
 * corresponding blk_pin_num.                                            *
 * [0...num_blocks-1][0...max_ports_per_blk-1][0...max_pins_per_port-1]  */
static int *** port_pin_to_blk_pin = NULL;

/* blk_pin_to_port_pin maps a blk_num and blk_pin_num to the             *
 * corresponding direct_index, if any. Otherwise, OPEN (-1) is stored.   *
 * [0...num_blocks-1][0...max_pins_per_clb-1]                            */
static int ** blk_pin_to_idirect = NULL;

/* blk_pin_to_port_pin maps a blk_num and blk_pin_num to the             *
 * corresponding direct_src_or_sink - SOURCE (0) is stored for a source, *
 * SINK (1) is stored for a sink, Otherwise, OPEN (-1) is stored.        *
 * [0...num_blocks-1][0...max_pins_per_clb-1]                            */
static int ** blk_pin_to_direct_src_or_sink = NULL;

/* The pl_macro_chains array stores all the carry chains.                *
 * [0...num_chains-1]                                                    */
static t_pl_macro * pl_macro_chains = NULL;
static int pl_macro_num_chains;

/*** These are the file scope subroutines declarations. ***/
static void free_placement_macros_structs(void);

/*** These are the file scope subroutines defintions. ***/
int alloc_and_load_placement_macros(t_direct_inf* directs, int num_directs, t_pl_macro ** chains){
	
	/* This function allocates and loads the chains placement macros   *
	 * and returns the total number of chains in 2 steps.              *
	 *   1) Allocate temporary data structure for maximum possible     *
	 *      size and loops through all the blocks storing the data     *
	 *      relevant to the carry chains. At the same time, also count *
	 *      the amount of memory required for the actual variables.    *
	 *   2) Allocate the actual variables with the exact amount of     *
	 *      memory. Then loads the data from the temporary data        *
	 *       structures before freeing them.                           */

	/* Declaration of local variables */
	int iblk, from_iblk_pin, to_iblk_pin, from_inet, to_inet, from_idirect, to_idirect;
	int from_src_or_sink, to_src_or_sink, next_iblk, next_iblk_pin, next_inet, next_src_or_sink;
	int num_blk_pins, num_chain; 
	int *pl_macro_idirect, *pl_macro_num_members, **pl_macro_member_blk_num;
	int ichain, imember;

	t_pl_macro * chain = NULL;
	
	/* Sets up the required variables. */
	port_pin_to_blk_pin = alloc_and_load_port_pin_to_blk_pin();
	alloc_and_load_blk_pin_to_port_pin(&blk_pin_to_port, &blk_pin_to_port_pin);
		
	alloc_and_load_blk_pin_to_idirect(directs, num_directs, port_pin_to_blk_pin,
			&blk_pin_to_idirect, &blk_pin_to_direct_src_or_sink);

	/* Allocate maximum memory for temporary variables. */
	num_chain = 0;
	pl_macro_num_members = (int *) my_calloc (num_blocks , sizeof(int));
	pl_macro_idirect = (int *) my_calloc (num_blocks , sizeof(int));
	pl_macro_member_blk_num = (int **) my_calloc (num_blocks , sizeof(int*));
	for (iblk = 0; iblk < num_blocks; iblk++) 
		pl_macro_member_blk_num[iblk] = (int *) my_calloc (num_blocks , sizeof(int));

	/* Compute required size:                                                *
	 * Go through all the pins with possible direct connections in           *
	 * blk_pin_to_idirect. Count the number of heads (which is the same      *
	 * as the number chains) and also the length of each chain               *
	 * Head - blocks with from_pin OPEN and to_pin connected                 *
	 * Tail - blocks with from_pin connected and to_pin OPEN                 */
	for (iblk = 0; iblk < num_blocks; iblk++) {

		num_blk_pins = block[iblk].type->num_pins;
		for (from_iblk_pin = 0; from_iblk_pin < num_blk_pins; from_iblk_pin++) {
			
			from_inet = block[iblk].nets[from_iblk_pin];
			from_idirect = blk_pin_to_idirect[iblk][from_iblk_pin];
			from_src_or_sink = blk_pin_to_direct_src_or_sink[iblk][from_iblk_pin];
			
			// Find from_pins with possible direct connection, make sure that it is not 
			// connected to any net
			if ( from_src_or_sink == SOURCE && from_idirect != OPEN && from_inet == OPEN ) {

				for (to_iblk_pin = 0; to_iblk_pin < num_blk_pins; to_iblk_pin++) {
					to_inet = block[iblk].nets[to_iblk_pin];
					to_idirect = blk_pin_to_idirect[iblk][to_iblk_pin];
					to_src_or_sink = blk_pin_to_direct_src_or_sink[iblk][to_iblk_pin];

					// Find to_pins with the same possible direct connection.
					if ( to_src_or_sink == SINK && to_idirect == from_idirect && to_inet != OPEN) {
						
						// Mark down the first block in the chain
						pl_macro_member_blk_num[num_chain][0] = iblk;
						pl_macro_idirect[num_chain] = to_idirect;
						
						// Also find out how many members are in the macros, there are at 
						// least 2 members - 1 head and 1 tail.
						next_inet = to_inet;
						do {
						
							// Make sure that it is connected to a net and only has 1 
							// sink - direct chain connection.
							assert(clb_net[next_inet].num_sinks == 1);
							
							// Increment the num_member count.
							pl_macro_num_members[num_chain]++;
							
							// Find the next block, make sure that they are consistent :-
							// 1) from the coord offsets specified in the arch file.
							// 2) from the netlist specified in the blif file.

							next_iblk = clb_net[next_inet].node_block[1];
							next_iblk_pin = clb_net[next_inet].node_block_pin[1];
							
							// Assert that the block connected next is indeed the block at the right offset.
							assert ( next_iblk == 
									grid[block[pl_macro_member_blk_num[num_chain][0]].x + directs[to_idirect].x_offset]
									[block[pl_macro_member_blk_num[num_chain][0]].y + directs[to_idirect].y_offset].blocks
									[block[pl_macro_member_blk_num[num_chain][0]].z + directs[to_idirect].z_offset]);
							
							// Find the following block connected.
							next_src_or_sink = blk_pin_to_direct_src_or_sink[next_iblk][next_iblk_pin];
							next_inet = block[next_iblk].nets[next_iblk_pin];
							
							// Mark down the next blk in the chain
							pl_macro_member_blk_num[num_chain][pl_macro_num_members[num_chain]] = next_iblk;

						} while(next_src_or_sink != OPEN);

						num_chain ++;

					} // Do nothing if the to_pins does not have same possible direct connection.
				} // Finish going through all the pins for to_pins.
			} // Do nothing if the from_pins does not have same possible direct connection.
		} // Finish going through all the pins for from_pins.
	} // Finish going through all blocks.
	// Now, all the data is readily stored in the temporary data structures.


	/* Allocate the memories for the chain. */
	chain = (t_pl_macro *) my_malloc (num_chain * sizeof(t_pl_macro));

	/* Allocate the memories for the chaim members.             *
	 * Load the values from the temporary data structures.      */
	for (ichain = 0; ichain < num_chain; ichain++) {
		chain[ichain].num_blocks = pl_macro_num_members[ichain];
		chain[ichain].members = (t_pl_macro_member *) my_malloc 
										(chain[ichain].num_blocks * sizeof(t_pl_macro_member));

		/* Load the values for each member of the chain */
		for (imember = 0; imember < chain[ichain].num_blocks; ) {
			chain[ichain].members[imember].x_offset = imember * directs[pl_macro_idirect[ichain]].x_offset;
			chain[ichain].members[imember].y_offset = imember * directs[pl_macro_idirect[ichain]].y_offset;
			chain[ichain].members[imember].z_offset = directs[pl_macro_idirect[ichain]].z_offset;
			chain[ichain].members[imember].blk_index = pl_macro_member_blk_num[ichain][imember];
		}
	}

	/* Frees up the temporary data structures. */
	free(pl_macro_num_members);
	free(pl_macro_idirect);
	free(pl_macro_member_blk_num);
	
	/* Keeps a static pointer to the chain. */
	pl_macro_chains = chain;
	pl_macro_num_chains = num_chain;
	
	/* Returns the pointer to the chain. */
	*chains = chain;
	return (num_chain);

}

int get_chain_index(int blk_num) {

	/* This function checks whether a block is part of a chain.  *
	 * If it is, returns the chain_index, otherwise, returns -1. */

	int ichain, imember, chain_index = -1;

	if (blk_num >= num_blocks || blk_num < 0)

	for (ichain = 0; ichain < pl_macro_num_chains; ichain ++) {
		for (imember = 0; imember < pl_macro_chains[ichain].num_blocks; imember++) {
			if (pl_macro_chains[ichain].members[imember].blk_index == blk_num)
				chain_index = ichain;
		}
	}

	return chain_index;
}

void free_placement_macros_structs(void) {

	/* This function frees up all the static data structures used. */

	int itype, iblk, ichain;

	// Calculate the size.
	int max_ports_per_blk = 0;
	for (itype = 0; itype < num_types; itype++) {
		max_ports_per_blk = max(max_ports_per_blk, type_descriptors[itype].pb_type->num_ports);
	}
	
	free_matrix(blk_pin_to_port, 0, num_blocks-1, 0, sizeof(int));
	free_matrix(blk_pin_to_port_pin, 0, num_blocks-1, 0, sizeof(int));
	free_matrix(blk_pin_to_idirect, 0, num_blocks-1, 0, sizeof(int));
	free_matrix(blk_pin_to_direct_src_or_sink, 0, num_blocks-1, 0, sizeof(int));
	
	// Special handling to free a 3D array.
	for (iblk = 0; iblk < num_blocks; iblk++)
		free_matrix(port_pin_to_blk_pin[iblk], 0, max_ports_per_blk, 0, sizeof(int));
	free(port_pin_to_blk_pin);
	
	// Special handling to free t_pl_macro array.
	for (ichain = 0; ichain < pl_macro_num_chains; ichain ++)
		free(pl_macro_chains[ichain].members);
	free(pl_macro_chains);
	
	// Set to NULL - defensive coding.
	pl_macro_chains = NULL;
	blk_pin_to_port = NULL;
	blk_pin_to_port_pin = NULL;
	blk_pin_to_idirect = NULL;
	blk_pin_to_direct_src_or_sink = NULL;
	port_pin_to_blk_pin = NULL;


}

//#endif