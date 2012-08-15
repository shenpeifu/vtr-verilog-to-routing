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