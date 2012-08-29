/****************************************************************************************
  Y.G.THIEN
  29 AUG 2012

    This file contains functions related to placement macros. The term "placement macros"
  refers to a structure that contains information on blocks that need special treatment
  during placement and possibly routing. 
  
    An example of placement macros is a carry chain. Blocks in a carry chain have to be 
  placed in a specific orientation or relative placement so that the carry_in's and the 
  carry_out's are properly aligned. With that, the carry chains would be able to use the 
  direct connections specified in the arch file. Direct connections with the pin's 
  fc_value 0 would be treated specially in routing where the whole carry chain would be
  treated as a unit and regular routing would not be used to connect the carry_in's and 
  carry_out's. Floorplanning constraints may also be an example of placement macros.

    The function alloc_and_load_placement_macros allocates and loads the placement 
  macros in the following steps:
  (1) First, go through all the block types and mark down the pins that could possibly 
      be part of a placement macros. 
  (2) Then, go through the netlist of all the pins marked in (1) to find out all the 
      heads of the placement macros using criteria depending on the type of placement 
	  macros. For carry chains, the heads of the placement macros are blocks with 
	  carry_in's not connected to any nets (OPEN) while the carry_out's connected to the 
	  netlist with only 1 SINK.
  (3) Traverse from the heads to the tails of the placement macros and load the 
      information in the t_pl_macro data structure. Similar to (2), tails are identified 
	  with criteria depending on the type of placement macros. For carry chains, the 
	  tails are blocks with carry_out's not connected to any nets (OPEN) while the 
	  carry_in's is connected to the netlist which has only 1 SINK.

    The only placement macros supported at the moment are the carry chains with limited
  functionality. 
    
	Current support for placement macros are:
  (1) The arch parser for direct connections is working. The specifications of the direct
      connections are specified in sample_adder_arch.xml and also in the 
	  VPR_User_Manual.doc
  (2) The placement macros allocator and loader is working.
  (3) The initial placement of placement macros that respects the restrictions of the 
      placement macros is working.
  (4) The post-placement legality check for placement macros is working.
    
	Current limitations on placement macros are:
  (1) One block could only be a part of a carry chain. In the future, if a block is part
      of multiple placement macros, we should load 1 huge placement macro instead of 
	  multiple placement macros that contain the same block.
  (2) Bus direct connections (direct connections with multiple bits) are supported. 
      However, a 2-bit carry chain when loaded would become 2 1-bit carry chains.
	  And because of (1), only 1 1-bit carry chain would be loaded. In the future, 
	  placement macros with multiple-bit connections or multiple 1-bit connections 
	  should be allowed.
  (3) Placement macros that span longer or wider than the chip would cause an error. 
      In the future, we *might* expand the size of the chip to accommodate such 
	  placement macros that are crucial.
  (4) For carry chain swap routine, the to_block could not be part of a placement macro.
      There is no limitation on the from_block. Currently, the carry chain swap routine
	  still has a bug in it.

****************************************************************************************/


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

/* blk_pin_to_port_pin maps a blk_num and blk_pin_num to the             *
 * corresponding direct_index, if any. Otherwise, OPEN (-1) is stored.   *
 * [0...num_types-1][0...num_blk_pins-1]                                 */
static int ** idirect_from_blk_pin = NULL;

/* blk_pin_to_port_pin maps a blk_num and blk_pin_num to the             *
 * corresponding direct_src_or_sink - SOURCE (0) is stored for a source, *
 * SINK (1) is stored for a sink, Otherwise, OPEN (-1) is stored.        *
 * [0...num_types-1][0...num_blk_pins-1]                             */
static int ** direct_type_from_blk_pin = NULL;

/* The pl_macro_chains array stores all the carry chains.                *
 * [0...num_chains-1]                                                    */
static t_pl_macro * pl_macro_chains = NULL;
static int pl_macro_num_chains;

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
	int iblk, from_iblk_pin, to_iblk_pin, from_inet, to_inet, from_idirect, to_idirect, 
			from_src_or_sink, to_src_or_sink;
	int next_iblk, curr_iblk, next_inet, curr_inet;
	int num_blk_pins, num_chain; 
	int ichain, imember;
	int *pl_macro_idirect, *pl_macro_num_members, **pl_macro_member_blk_num;
	
	t_pl_macro * chain = NULL;
	
	/* Sets up the required variables. */
	alloc_and_load_idirect_from_blk_pin(directs, num_directs, 
			&idirect_from_blk_pin, &direct_type_from_blk_pin);

	/* Allocate maximum memory for temporary variables. */
	pl_macro_num_members = (int *) my_calloc (num_blocks , sizeof(int));
	pl_macro_idirect = (int *) my_calloc (num_blocks , sizeof(int));
	pl_macro_member_blk_num = (int **) my_calloc (num_blocks , sizeof(int*));
	for (iblk = 0; iblk < num_blocks; iblk++) 
		pl_macro_member_blk_num[iblk] = (int *) my_calloc (num_blocks , sizeof(int));

	/* Compute required size:                                                *
	 * Go through all the pins with possible direct connections in           *
	 * idirect_from_blk_pin. Count the number of heads (which is the same      *
	 * as the number chains) and also the length of each chain               *
	 * Head - blocks with to_pin OPEN and from_pin connected                 *
	 * Tail - blocks with to_pin connected and from_pin OPEN                 */
	num_chain = 0;
	for (iblk = 0; iblk < num_blocks; iblk++) {

		num_blk_pins = block[iblk].type->num_pins;
		for (to_iblk_pin = 0; to_iblk_pin < num_blk_pins; to_iblk_pin++) {
			
			to_inet = block[iblk].nets[to_iblk_pin];
			to_idirect = idirect_from_blk_pin[block[iblk].type->index][to_iblk_pin];
			to_src_or_sink = direct_type_from_blk_pin[block[iblk].type->index][to_iblk_pin];
			
			// Find to_pins (SINKs) with possible direct connection but are not 
			// connected to any net
			if ( to_src_or_sink == SINK && to_idirect != OPEN && to_inet == OPEN ) {

				for (from_iblk_pin = 0; from_iblk_pin < num_blk_pins; from_iblk_pin++) {
					from_inet = block[iblk].nets[from_iblk_pin];
					from_idirect = idirect_from_blk_pin[block[iblk].type->index][from_iblk_pin];
					from_src_or_sink = direct_type_from_blk_pin[block[iblk].type->index][from_iblk_pin];

					// Find from_pins with the same possible direct connection that are connected.
					if ( from_src_or_sink == SOURCE && to_idirect == from_idirect && from_inet != OPEN) {
						
						// Mark down that this is the first block in the chain
						pl_macro_member_blk_num[num_chain][0] = iblk;
						pl_macro_idirect[num_chain] = to_idirect;
						
						// Increment the num_member count.
						pl_macro_num_members[num_chain]++;
						
						// Also find out how many members are in the macros, 
						// there are at least 2 members - 1 head and 1 tail.
						
						// Initialize the variables
						next_inet = from_inet;
						next_iblk = iblk;

						// Start finding the other members
						while (next_inet != OPEN) {

							curr_iblk = next_iblk;
							curr_inet = next_inet;
							
							// Assume that carry chains only has 1 sink - direct connection
							assert(clb_net[curr_inet].num_sinks == 1);
							next_iblk = clb_net[curr_inet].node_block[1];
							
							// Assume that the from_iblk_pin index is the same for the next block
							assert (idirect_from_blk_pin[block[next_iblk].type->index][from_iblk_pin] == from_idirect
									&& direct_type_from_blk_pin[block[next_iblk].type->index][from_iblk_pin] == SOURCE);
							next_inet = block[next_iblk].nets[from_iblk_pin];

							// Mark down this block as a member of the chain
							imember = pl_macro_num_members[num_chain];
							pl_macro_member_blk_num[num_chain][imember] = next_iblk;

							// Increment the num_member count.
							pl_macro_num_members[num_chain]++;

						} // Found all the members of this chain at this point


						// Increment the chain count
						num_chain ++;

					} // Do nothing if the from_pins does not have same possible direct connection.
				} // Finish going through all the pins for from_pins.
			} // Do nothing if the to_pins does not have same possible direct connection.
		} // Finish going through all the pins for to_pins.
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
		for (imember = 0; imember < chain[ichain].num_blocks; imember++) {
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

	int itype, ichain;

	// Calculate the size.
	int max_ports_per_blk = 0;
	for (itype = 0; itype < num_types; itype++) {
		max_ports_per_blk = max(max_ports_per_blk, type_descriptors[itype].pb_type->num_ports);
	}
	
	free_matrix(idirect_from_blk_pin, 0, num_types-1, 0, sizeof(int));
	free_matrix(direct_type_from_blk_pin, 0, num_types-1, 0, sizeof(int));
	
	// Special handling to free t_pl_macro array.
	for (ichain = 0; ichain < pl_macro_num_chains; ichain ++)
		free(pl_macro_chains[ichain].members);
	free(pl_macro_chains);
	
	// Set to NULL - defensive coding.
	pl_macro_chains = NULL;
	idirect_from_blk_pin = NULL;
	direct_type_from_blk_pin = NULL;


}
