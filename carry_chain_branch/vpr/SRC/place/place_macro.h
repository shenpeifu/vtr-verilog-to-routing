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


#ifndef PLACE_MACRO_H
#define PALCE_MACRO_H

/* These are the placement macro structure. 
 * It is in the form of array of structs instead of
 * structs of arrays for cache efficiency. 
 * Could have more data members for other macro type.
 * blk_index: The block index of this block. 
 * x_offset: The x_offset of the previous block to this block. 
 * y_offset: The y_offset of the previous block to this block. 
 */
typedef struct s_pl_macro_member{
	int blk_index;
	int x_offset;
	int y_offset;	 
	int z_offset;
} t_pl_macro_member;

/* num_blocks: The number of blocks this macro contains.
 * members: An array of blocks in this macro [0¡­num_macro-1].
 * idirect: The direct index as specified in the arch file
 */
typedef struct s_pl_macro{
	int num_blocks;
	t_pl_macro_member* members;
} t_pl_macro;

/* These are the function declarations. */
int alloc_and_load_placement_macros(t_direct_inf* directs, int num_directs, t_pl_macro ** chains);
int get_chain_index(int blk_num);
void free_placement_macros_structs(void);

#endif