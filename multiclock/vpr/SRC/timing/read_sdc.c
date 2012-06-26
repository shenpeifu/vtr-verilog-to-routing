#include <string.h>
#include <stdio.h>
#include <limits.h>
#include "assert.h"
#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "read_sdc.h"
#include "read_blif.h"
#include "path_delay.h"

/****************** Variables local to this module **************************/

static FILE *sdc;
t_sdc_clock * sdc_clocks = NULL; /* List of clock periods and offsets, read *
								  * in directly from SDC file				*/
int * exclusive_clock_groups = NULL; /* Temporary structure used to interpret   *
									  * the command set_clock_groups -exclusive */

int num_netlist_clocks; /* number of clocks in netlist */
char ** netlist_clocks; /* [0..num_netlist_clocks - 1] array of names of clocks in netlist */

int num_netlist_ios; /* number of clocks in netlist */
char ** netlist_ios; /* [0..num_netlist_clocks - 1] array of names of clocks in netlist */

/***************** Subroutines local to this module *************************/

static void alloc_and_load_netlist_clocks_and_ios(void);
static void count_netlist_clocks(void);
static void get_sdc_tok(char * buf, int num_lines);
static boolean check_if_number(char * ptr);
static int find_clock(char * ptr);
static float calculate_constraint(t_sdc_clock source_domain, t_sdc_clock sink_domain);

/********************* Subroutine definitions *******************************/

void read_sdc(char * sdc_file) {
/* This function reads the constraints from the SDC file specified on  *
 * the command line into float ** timing_constraint.  If no file is   *
 * specified, it leaves timing_constraint pointing to NULL.            */

	char buf[BUFSIZE];
	int num_lines = 1; /* Line counter for SDC file, used to report errors */
	int source_clock_domain, sink_clock_domain;
	
	/* If no SDC file is included or specified, use default behaviour of cutting paths between domains and optimizing each clock separately */
	if ((sdc = fopen(sdc_file, "r")) == NULL) {
		printf("\nSDC file %s not found.\n", sdc_file);
		printf("All clocks will be optimized to run as fast as possible.\n");
		printf("Paths between clock domains will be cut.\n\n");

		/* Find the number of clocks with constraints from the netlist. */
		count_netlist_clocks();

		/* Allocate matrix of timing constraints [0..num_netlist_clocks-1][0..num_netlist_clocks-1] */
		timing_constraint = (float **) alloc_matrix(0, num_netlist_clocks-1, 0, num_netlist_clocks-1, sizeof(float));
		for(source_clock_domain=0; source_clock_domain<num_netlist_clocks; source_clock_domain++) {
			for(sink_clock_domain=0; sink_clock_domain<num_netlist_clocks; sink_clock_domain++) {
				if(source_clock_domain == sink_clock_domain) {
					timing_constraint[source_clock_domain][sink_clock_domain] = 0.;
				} else {
					timing_constraint[source_clock_domain][sink_clock_domain] = DO_NOT_ANALYSE;
				}
			}
		}
		return;
	}
	
	/* Now we have an SDC file. */

	/* Count how many clocks and I/Os are in the netlist. 
	Store the names of each clock and each I/O in netlist_clocks and netlist_ios. 
	The only purpose of these two lists is to compare clock names in the SDC file against them.
	As a result, they will be freed after the SDC file is parsed. */
	alloc_and_load_netlist_clocks_and_ios();



	/* Parse the file line-by-line. */
	while (my_fgets(buf, BUFSIZE, sdc) != NULL) { 
		get_sdc_tok(buf, num_lines);
		num_lines++;
	}
	
	fclose(sdc);

	/* Allocate matrix of timing constraints [0..num_constrained_clocks-1][0..num_constrained_clocks-1] and initialize to 0 */
	timing_constraint = (float **) alloc_matrix(0, num_constrained_clocks-1, 0, num_constrained_clocks-1, sizeof(float));
	
	/* Based on the information from sdc_clocks, calculate constraints for all paths not excluded by set_false_path or set_clock_groups -exclusive */
		for(source_clock_domain=0; source_clock_domain<num_constrained_clocks; source_clock_domain++) {
			for(sink_clock_domain=0; sink_clock_domain<num_constrained_clocks; sink_clock_domain++) {
				if(timing_constraint[source_clock_domain][sink_clock_domain] != DO_NOT_ANALYSE)
					timing_constraint[source_clock_domain][sink_clock_domain] = calculate_constraint(sdc_clocks[source_clock_domain], sdc_clocks[sink_clock_domain]);
			}
		}

	/* Since all the information we need is stored in timing_constraint, free other data structures used in this routine */
	free(sdc_clocks);
	return;
}

static void alloc_and_load_netlist_clocks_and_ios(void) {

	/* Count how many clocks and I/Os are in the netlist. 
	Store the names of each clock and each I/O in netlist_clocks and netlist_ios. */

	int iblock, i, net;
	char * name;
	boolean found;

	num_netlist_clocks = 0;
	num_netlist_ios = 0;

	for (iblock = 0; iblock < num_logical_blocks; iblock++) {
		if(logical_block[iblock].type == VPACK_LATCH) {
			net = logical_block[iblock].clock_net;
			name = logical_block[net].name;
			/* Now that we've found a clock, let's see if we've counted it already */
			found = FALSE;
			for (i = 0; !found && i < num_netlist_clocks; i++) {
				if (strcmp(netlist_clocks[i], name) == 0) {
					found = TRUE;
				}
			}
			if(!found) {
			/* If we get here, the clock is new and so we add it to the netlist_clocks */
				num_netlist_clocks++;
				/*dynamically grow the array to fit one new element */
				netlist_clocks = (char **) my_realloc (netlist_clocks, num_netlist_clocks * sizeof(t_clock));
				netlist_clocks[num_netlist_clocks - 1] = name;
			}
		} else if(logical_block[iblock].type == VPACK_INPAD || logical_block[iblock].type == VPACK_OUTPAD) {
			net = logical_block[iblock].clock_net;
			name = logical_block[net].name;
			/* Now that we've found an I/O, let's see if we've counted it already */
			found = FALSE;
			for (i = 0; !found && i < num_netlist_ios; i++) {
				if (strcmp(netlist_ios[i], name) == 0) {
					found = TRUE;
				}
			}
			if(!found) {
			/* If we get here, the clock is new and so we add it to the netlist_clocks */
				num_netlist_ios++;
				/*dynamically grow the array to fit one new element */
				netlist_ios = (char **) my_realloc (netlist_ios, num_netlist_ios * sizeof(t_io));
				netlist_ios[num_netlist_ios - 1] = name;
			}
		}
	}
}

static void count_netlist_clocks(void) {
	/* Count how many clocks are in the netlist. */

	int iblock, i, net;
	char * name;
	boolean found;

	num_netlist_clocks = 0;
	num_netlist_ios = 0;

	for (iblock = 0; iblock < num_logical_blocks; iblock++) {
		if(logical_block[iblock].type == VPACK_LATCH) {
			net = logical_block[iblock].clock_net;
			name = logical_block[net].name;
			/* Now that we've found a clock, let's see if we've counted it already */
			found = FALSE;
			for (i = 0; !found && i < num_netlist_clocks; i++) {
				if (strcmp(netlist_clocks[i], name) == 0) {
					found = TRUE;
				}
			}
			if(!found) {
			/* If we get here, the clock is new and so we add it to the netlist_clocks */
				num_netlist_clocks++;
			}
		}
	}
}

static void get_sdc_tok(char * buf, int num_lines) {
/* Figures out which, if any, token is at the start of this line and takes the appropriate action. */

#define SDC_TOKENS " \t\n{}[]" 
	/* We're using so little of the SDC syntax that we can ignore braces */

	char * ptr;
	float clock_period, rising_edge, falling_edge, max_io_delay;
	int source_clock_domain, sink_clock_domain, index, from, to, num_exclusive_groups = 0;

	/* my_strtok splits the string into tokens - little character arrays separated by the SDC_TOKENS defined above.          *
	 * Throughout this code, ptr refers to the tokens we fetch, one at a time.  The token changes at each call of my_strtok. */
	ptr = my_strtok(buf, SDC_TOKENS, sdc, buf);

	if (!ptr) { /* blank line */
		return;
	}

	if (ptr[0] == '#') { /* line starts with a comment */
		return; 
	}

	if (strcmp(ptr, "create_clock") == 0) {
		/* Syntax: create_clock -period <float> [-waveform {rising_edge falling_edge}] <clock names> */

		/* make sure clock has -period specified */
		/* Note: we call my_strtok with NULL as the first argument every time AFTER the first, since this picks up tokenizing where we left off */
		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);

		if(strcmp(ptr, "-period") != 0) {
			fprintf(stderr, "Create_clock must be followed by '-period' on line %d of SDC file.\n", num_lines);
			exit(1);
		}
		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);

		/* Check if the token following -period is actually a number. */
		if(!check_if_number(ptr)) {
			fprintf(stderr, "Token following '-period' is not a number on line %d of SDC file.\n", num_lines);
			exit(1);
		}
		
		clock_period = (float) strtod(ptr, NULL);

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(!ptr) {
			fprintf(stderr, "Clock net(s) not specified at end of line %d of SDC file.\n", num_lines);
			exit(1);
		}
		
		if(strcmp(ptr, "-waveform") == 0) {
			/* Get the first float, which is the rising edge, and the second, which is the falling edge. */
			ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
			if(!check_if_number(ptr)) {
				fprintf(stderr, "First token following '-waveform' is not a number on line %d of SDC file.\n", num_lines);
				exit(1);
			}
			rising_edge = (float) strtod(ptr, NULL);
			ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
			if(!check_if_number(ptr)) {
				fprintf(stderr, "Second token following '-waveform' is not a number on line %d of SDC file.\n", num_lines);
				exit(1);
			}
			falling_edge = (float) strtod(ptr, NULL);
			/* Check that the falling edge is one half period away from the rising edge, excluding rounding error. */
			if(abs(rising_edge - falling_edge) - clock_period/2.0 > EQUAL_DEF) {
				fprintf(stderr, "Clock does not have 50%% duty cycle on line %d of SDC file.\n", num_lines);
				exit(1);
			}
			ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf); /* We need this extra one to advance the ptr to the right spot. */
		} else {
			/* The clock's rising edge is by default at 0, and the falling edge is at the half-period. */
			rising_edge = 0.;
			falling_edge = clock_period / 2.0;
		}

		if (strcmp(ptr, "-name") == 0) {
			/* For external virtual clocks only (used with I/O constraints).  
			Only one virtual clock can be specified per line, 
			so make sure there's only one token left on this line. */

			/* Get the virtual clock name */
			ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);

			/* Store the clock's name, period and offset in the local array sdc_clocks. */
			sdc_clocks = (t_sdc_clock *) my_realloc(sdc_clocks, num_constrained_clocks);
			sdc_clocks[num_constrained_clocks - 1].name = my_strdup(ptr);
			sdc_clocks[num_constrained_clocks - 1].period = clock_period;
			sdc_clocks[num_constrained_clocks - 1].offset = rising_edge; 

			/* The next token should be NULL.  If so, return; if not, print an error message and exit. */
			if (ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf)) {
				fprintf(stderr, "Only one virtual clock name can be specified after -name on line %d of SDC file.\n", ptr, num_lines);
				exit(1);
			}
			return;

		} else {
			/* Parse through to the end of the line.  All that should be left on this line are a bunch of 
			 * clock nets to be associated with this clock period.  An array sdc_clocks will
			 * store the period and offset of each clock at the same index which that clock has in netlist_clocks.
			 * After everything has been parsed, we take the information from this array to calculate the actual timing constraints
			 * which these periods and offsets imply, and put them in the matrix timing_constraint. */

			for(;;) {
				if(!ptr) { /* end of line */
					return; 
				}

				if (find_clock(ptr) == -1) {
					fprintf(stderr, "Clock name %s does not correspond to a net on line %d of SDC file.\n" 
						"If you'd like to create a virtual clock, use the -name keyword.\n", ptr, num_lines);
					exit(1);
				}

				/* We've found a new clock! */
				num_constrained_clocks++;

				sdc_clocks = (t_sdc_clock *) my_realloc(sdc_clocks, num_constrained_clocks);
				sdc_clocks[num_constrained_clocks - 1].name = my_strdup(ptr);
				sdc_clocks[num_constrained_clocks - 1].period = clock_period;
				sdc_clocks[num_constrained_clocks - 1].offset = rising_edge; 

				constrained_clocks = (t_clock *) my_realloc (constrained_clocks, num_constrained_clocks);
				constrained_clocks[num_constrained_clocks - 1].name = my_strdup(ptr);
				/* Fanout will be filled out once the timing graph has been constructed. */
				
				/* Advance to the next token (or the end of the line). */
				ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
			}	
		}
	} else if (strcmp(ptr, "set_clock_groups") == 0) {
		/* Syntax: set_clock_groups -exclusive -group {<clock name(s)>} -group {<clock name(s)>} [-group {<clock name(s)>} ...] */

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-exclusive") != 0) {
			fprintf(stderr, "Set_clock_groups must be followed by '-exclusive' on line %d of SDC file.\n", num_lines);
			exit(1);
		}
		
		/* Parse through to the end of the line.  All that should be left on this line are a bunch of 
		 * -group commands, followed by lists of clocks in that group.  An array exclusive_clock_groups will
		 * store the group number of each clock at the index location of that clock in netlist_clocks - i.e. if the clock
		 * at index 5 in netlist_clocks appears after the second -group token, then exclusive_clock_groups[5] = 2. 
		 * Finally, we set timing_constraint to not analyse between clock domains with different group numbers.  */
		for(;;) {
			ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
			if(ptr == NULL) { /* end of line */
				break; /* exit the infinite for loop - but don't return yet - we still have to populate timing_constraint!  */
			}
			if(strcmp(ptr, "-group") == 0) {
			/* add 1 to the group number we're assigning clocks to every time the token -group is hit */
				num_exclusive_groups++;
			} else { 
				/* Presumptively, we have a clock. Calloc the array exclusive_clock_groups if not done already, then *
				 * find the index of the clock named by ptr, and set exclusive_clock_groups[that clock's index] equal to the current group number */
				if(!exclusive_clock_groups) {
					exclusive_clock_groups = (int *) my_calloc(num_netlist_clocks, sizeof(int));
				}
				index = find_clock(ptr);
				if (index == -1) {
					fprintf(stderr, "Clock name %s does not correspond to a net on line %d of SDC file.\n", ptr, num_lines);
					exit(1);
				}
				exclusive_clock_groups[index] = num_exclusive_groups;
			}
		}

		/* Finally, set every element of timing_constraint for which the two indices have different group numbers to DO_NOT_ANALYSE */
		for(source_clock_domain=0; source_clock_domain<num_netlist_clocks; source_clock_domain++) {
			for(sink_clock_domain=0; sink_clock_domain<num_netlist_clocks; sink_clock_domain++) {
				/* if either source or sink domain is part of group 0 (i.e. not part of an exclusive group), don't touch it */
				if(exclusive_clock_groups[source_clock_domain] != 0 && exclusive_clock_groups[sink_clock_domain] != 0) {
					if(exclusive_clock_groups[source_clock_domain] != exclusive_clock_groups[sink_clock_domain]) {
						timing_constraint[source_clock_domain][sink_clock_domain] = DO_NOT_ANALYSE;
					}
				}
			}
		}
		free(exclusive_clock_groups);
		return;

	} else if (strcmp(ptr, "set_false_path") == 0) {
		/* Syntax: set_false_path -from <clock name> to <clock name> */

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-from") != 0) {
			fprintf(stderr, "Set_false_path must be followed by '-from <clock_name>' on line %d of SDC file.\n", num_lines);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		from = find_clock(ptr);
		if (from == -1) {
			fprintf(stderr, "Clock name %s does not correspond to a net on line %d of SDC file.\n", ptr, num_lines);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-to") != 0) {
			fprintf(stderr, "Set_false_path must be followed by '-from <clock_name> -to <clock_name> on line %d of SDC file.\n", num_lines);
			exit(1);
		}
		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		to = find_clock(ptr);
		if (to == -1) {
			fprintf(stderr, "Clock name %s does not correspond to a net on line %d of SDC file.\n", ptr, num_lines);
			exit(1);
		}

		/* Set the path from 'from' to 'to' in timing_constraint to not be analysed */
		timing_constraint[from][to] = DO_NOT_ANALYSE;
		
		return;

	} else if (strcmp(ptr, "set_input_delay") == 0) {
		/* Syntax: set_input_delay -clock <virtual_clock> -max <max_io_delay> [get_ports {<port_list>}]
		We want to assign virtual_clock to all ports in port_list, and set the delay from the input 
		(between devices) to io_delay. */

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-clock") != 0) {
			fprintf(stderr, "Set_input_delay must be followed by '-clock <virtual_clock_name>' on line %d of SDC file.\n", num_lines);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		index = find_clock(ptr);
		if (index == -1) {
			/* This set_input_delay command is above the corresponding create_clock command in the SDC file, so it's a new clock. 
			Expand netlist_clocks and sdc_clocks to fit one new element.  We'll fill in the details of sdc_clocks when we find the create_clock command. */
			num_constrained_clocks++;
			index = num_constrained_clocks - 1;
			netlist_clocks = (t_clock *) my_realloc (netlist_clocks, num_netlist_clocks * sizeof(t_clock));
			netlist_clocks[index].name = my_strdup(ptr);
			netlist_clocks[index].fanout = 0;
			sdc_clocks = (t_sdc_clock *) my_realloc (sdc_clocks, num_netlist_clocks * sizeof(t_sdc_clock));
		} else {
			/* This set_input_delay command is below the corresponding create_clock command in the SDC file.
				The variable index already stores the index of the virtual clock in both netlist_clocks and sdc_clocks. */
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-max") != 0) {
			fprintf(stderr, "Set_input_delay must be followed by '-max <maximum_input_delay>' on line %d of SDC file.\n", num_lines);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		/* check if the token following -max is actually a number*/
		if(!check_if_number(ptr)) {
			fprintf(stderr, "Token following '-max' is not a number on line %d of SDC file.\n", num_lines);
			exit(1);
		}
		max_io_delay = (float) strtod(ptr, NULL);

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "get_ports") != 0) {
			fprintf(stderr, "Set_input_delay requires a [get_ports {...}] command on line %d of SDC file.\n", num_lines);
			exit(1);
		}

		/* Parse through to the end of the line.  Add each name we find to the list of constrained I/Os
			along with the virtual clock name and max_io_delay we've just parsed. */
		


		return;

	} else if (strcmp(ptr, "set_output_delay") == 0) {
		/* Format: set_input_delay -clock <virtual_clock> -max <io_delay> [get_ports {<port_list>}]
		We want to assign virtual_clock to all ports in port_list, and set the delay to the output 
		(between devices) to io_delay */

		return;

	} else {
		fprintf(stderr, "Incorrect or unsupported syntax near start of line %d of SDC file.\n", num_lines);
		exit(1);
	}
}

static boolean check_if_number(char * ptr) {
/* Checks if the character array ptr represents a valid floating-point number.  *
 * To return TRUE, all characters must be digits, although *
 * there can also be no more than one decimal point.       */
	int i, len, num_decimal_points = 0;
	len = strlen(ptr);
	for (i=0; i<len; i++) {
		if((ptr[i] < '0' || ptr[i] > '9')) {
			if(ptr[i] != '.') {
				return FALSE;
			}
			num_decimal_points++;
			if(num_decimal_points > 1) {
				return FALSE;
			}
		}
	}
	return TRUE;
}

static int find_clock(char * ptr) {
/* Given a string clock_name, find whether it's the name of a clock in the array netlist_clocks.  *
 * if it is, return the clock's index in netlist_clocks; if it's not, return -1. */
	int index;
	for(index=0;index<num_netlist_clocks;index++) {
		if(strcmp(ptr, netlist_clocks[index]) == 0) {
			return index;
		}
	}
	return -1;
}

static float calculate_constraint(t_sdc_clock source_domain, t_sdc_clock sink_domain) {
	/* Given information from the SDC file about the period and offset of two clocks, *
	* determine the implied setup-time constraint between them via edge counting.     */

	int source_period, sink_period, source_offset, sink_offset, lcm_period, num_source_edges, num_sink_edges, 
		* source_edges, * sink_edges, i, j, time, constraint_as_int;
	float constraint;

	/* If the source and sink domains are the same, the constraint is just the clock period. */
	if((source_domain.period == sink_domain.period) && (source_domain.offset == sink_domain.offset)) {
		constraint = source_domain.period; /* or, equivalently, sink_domain.period */
		return constraint;
	}
	
	/* If we get here, the two constraints are not the same, and so we have to use edge counting. *
	 * Multiply periods and offsets by 1000 and round down to the nearest integer,				  *
	 * to avoid messy decimals. */

	source_period = (int) source_domain.period * 1000;
	sink_period = (int) sink_domain.period * 1000;
	source_offset = (int) source_domain.offset * 1000;	
	sink_offset = (int) sink_domain.offset * 1000;

	/* Find the LCM of the two periods.  *
	* This determines how long it takes before the pattern of the two clocks starts repeating. */
	for(lcm_period = 1; ; lcm_period++) {
		if(lcm_period % source_period == 0 && lcm_period % sink_period == 0) {
			break;
		}
	}

	/* Create an array of positive edges for each clock over one LCM clock period. */

	num_source_edges = lcm_period/source_period + 1; 
	num_sink_edges = lcm_period/sink_period + 1;

	source_edges = (int *) my_malloc((num_source_edges + 1) * sizeof(int));
	sink_edges = (int *) my_malloc((num_sink_edges + 1) * sizeof(int));
	
	for(i=0, time=source_offset; i<num_source_edges; i++) {
		source_edges[i] = time;
		time += source_period;
	}

	for(i=0, time=sink_offset; i<num_sink_edges; i++) {
		sink_edges[i] = time;
		time += sink_period;
	}

	/* Compare every edge in source_edges with every edge in sink_edges.			 *
	 * The lowest STRICTLY POSITIVE difference between a sink edge and a source edge *
	 * gives us the set-up time constraint.											 */

	constraint_as_int = INT_MAX; /* constraint starts off at +ve infinity so that everything will be less than it */

	for(i=0; i<num_source_edges+1; i++) {
		for(j=0; j<num_sink_edges+1; j++) {
			if(sink_edges[j]>source_edges[i]) {
				constraint_as_int = min(constraint_as_int, sink_edges[j]-source_edges[i]);
			}
		}
	}

	/* Divide by 1000 again and turn the constraint back into a float, and clean up memory. */

	constraint = (float) constraint_as_int/1000;

	free(source_edges);
	free(sink_edges);

	return constraint;
}