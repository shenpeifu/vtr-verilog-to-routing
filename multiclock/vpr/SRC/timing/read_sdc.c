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
#include "ReadOptions.h"
#include "regex.h"

/****************** Types local to this module **************************/

typedef struct s_sdc_clock {
	char * name;
	float period;
	float offset;
} t_sdc_clock;
/* Stores the name, period and offset of each constrained clock. */

typedef struct s_sdc_override_constraint {
	char * source_clock_domain;
	char * sink_clock_domain;
	float constraint;
	int num_multicycles;
} t_sdc_override_constraint;
/* A special-case constraint to override the default, calculated, timing constraint.
Holds data from set_clock_groups, set_false_path, set_max_delay, and set_multicycle_path
commands. */

typedef struct s_sdc_exclusive_group {
	char * name;
	int group;
} t_sdc_exclusive_group;
/* Used to temporarily separate clock names into exclusive groups when parsing the 
command set_clock_groups -exclusive. */

/****************** Variables local to this module **************************/

static FILE *sdc;
t_sdc_clock * sdc_clocks = NULL; /* List of clock periods and offsets, read *
								  * in directly from SDC file				*/

int num_netlist_clocks = 0; /* number of clocks in netlist */
char ** netlist_clocks; /* [0..num_netlist_clocks - 1] array of names of clocks in netlist */

int num_netlist_ios = 0; /* number of clocks in netlist */
char ** netlist_ios; /* [0..num_netlist_clocks - 1] array of names of clocks in netlist */

int num_override_constraints = 0; /* number of special-case constraint to override default, calculated, timing constraints */
t_sdc_override_constraint * override_constraints; /*  [0..num_override_constraints - 1] array of such constraints */

/***************** Subroutines local to this module *************************/

static void alloc_and_load_netlist_clocks_and_ios(void);
static void count_netlist_clocks_as_constrained_clocks(void);
static void get_sdc_tok(char * buf);
static boolean check_if_number(char * ptr);
static int find_constrained_clock(char * ptr);
static float calculate_constraint(t_sdc_clock source_domain, t_sdc_clock sink_domain);
static int find_override_constraint(int source_clock_domain, int sink_clock_domain);
static void print_timing_constraint_info(char *fname);
static void print_spaces(FILE * fp, int num_spaces);
static boolean regex_match (char *string, char *pattern);

/********************* Subroutine definitions *******************************/

void read_sdc(char * sdc_file) {
/* This function reads the constraints from the SDC file specified on  *
 * the command line into float ** timing_constraint.  If no file is   *
 * specified, it leaves timing_constraint pointing to NULL.            */

	char buf[BUFSIZE];
	int source_clock_domain, sink_clock_domain, io_index, override_index;
	float constraint;
	
	/* Make sure we haven't called this subroutine before. */
	assert(!timing_constraint);

	/* Reset file line number. */
	file_line_number = 0;

	/* If no SDC file is included or specified, use default behaviour of cutting paths between domains and optimizing each clock separately */
	if ((sdc = fopen(sdc_file, "r")) == NULL) {
		printf("SDC file %s not found.\n", sdc_file);
		printf("All clocks will be optimized to run as fast as possible.\n");
		printf("Paths between clock domains will be cut.\n\n");

		/* Find all netlist clocks and add them as constrained clocks. */
		count_netlist_clocks_as_constrained_clocks();

		/* Allocate matrix of timing constraints [0..num_constrained_clocks-1][0..num_constrained_clocks-1] */
		timing_constraint = (float **) alloc_matrix(0, num_constrained_clocks-1, 0, num_constrained_clocks-1, sizeof(float));
		for(source_clock_domain=0; source_clock_domain<num_constrained_clocks; source_clock_domain++) {
			for(sink_clock_domain=0; sink_clock_domain<num_constrained_clocks; sink_clock_domain++) {
				if(source_clock_domain == sink_clock_domain) {
					timing_constraint[source_clock_domain][sink_clock_domain] = 0.;
				} else {
					timing_constraint[source_clock_domain][sink_clock_domain] = DO_NOT_ANALYSE;
				}
			}
		}

		if (GetEchoOption()) {
			print_timing_constraint_info("timing_constraints.echo");
		}

		/* Now normalize timing_constraint and constrained_ios to be in seconds, not nanoseconds. */
		for (source_clock_domain=0; source_clock_domain<num_constrained_clocks; source_clock_domain++) {
			for (sink_clock_domain=0; sink_clock_domain<num_constrained_clocks; sink_clock_domain++) {
				constraint = timing_constraint[source_clock_domain][sink_clock_domain];
				if (constraint > -0.01) { /* if constraint does not equal DO_NOT_ANALYSE */
					timing_constraint[source_clock_domain][sink_clock_domain] = constraint/1e9;
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
		get_sdc_tok(buf);
	}
	
	fclose(sdc);

	/* Make sure that all virtual clocks referenced in set_input_delay and set_output_delay have been constrained. */
	for (io_index = 0; io_index < num_constrained_ios; io_index++) {
		if ((find_constrained_clock(constrained_ios[io_index].virtual_clock_name)) == -1) {
			fprintf(stderr, "I/O %s is associated with an unconstrained clock %s in SDC file.\n", constrained_ios[io_index].name, constrained_ios[io_index].virtual_clock_name);
			exit(1);
		}
	}

	/* Make sure that all clocks referenced in override_constraints have been constrained. */
	for (override_index = 0; override_index < num_override_constraints; override_index++) {
		if ((find_constrained_clock(override_constraints[override_index].source_clock_domain)) == -1) {
			fprintf(stderr, "Token %s is not a clock constrained in the SDC file.\n", override_constraints[override_index].source_clock_domain);
			exit(1);
		} else if (find_constrained_clock(override_constraints[override_index].sink_clock_domain) == -1) {
			fprintf(stderr, "Token %s is not a clock constrained in the SDC file.\n", override_constraints[override_index].sink_clock_domain);
			exit(1);
		}
	}

	/* Allocate matrix of timing constraints [0..num_constrained_clocks-1][0..num_constrained_clocks-1] and initialize to 0 */
	timing_constraint = (float **) alloc_matrix(0, num_constrained_clocks-1, 0, num_constrained_clocks-1, sizeof(float));
	
	/* Based on the information from sdc_clocks, calculate constraints for all paths except ones with an override constraint. */
		for (source_clock_domain=0; source_clock_domain<num_constrained_clocks; source_clock_domain++) {
			for (sink_clock_domain=0; sink_clock_domain<num_constrained_clocks; sink_clock_domain++) {
				if ((override_index = find_override_constraint(source_clock_domain, sink_clock_domain)) != -1) {
					if (override_constraints[override_index].num_multicycles == 0) {
						/* There's a special constraint from set_false_path, set_clock_groups -exclusive or set_max_delay which overrides the default constraint. */
						timing_constraint[source_clock_domain][sink_clock_domain] = override_constraints[override_index].constraint;
					} else {
						/* There's a special constraint from set_multicycle_path which overrides the default constraint. 
						This constraint = default constraint (obtained via edge counting) + (num_multicycles - 1) * period of sink clock domain. */
						timing_constraint[source_clock_domain][sink_clock_domain] = 
							calculate_constraint(sdc_clocks[source_clock_domain], sdc_clocks[sink_clock_domain]) + (override_constraints[override_index].num_multicycles - 1)* sdc_clocks[sink_clock_domain].period;
					}
				} else {
					/* Calculate the constraint between clock domains by finding the smallest positive difference between a posedge in the source domain and one in the sink domain. */
					timing_constraint[source_clock_domain][sink_clock_domain] = calculate_constraint(sdc_clocks[source_clock_domain], sdc_clocks[sink_clock_domain]);
				}
			}
		}

	if (GetEchoOption()) {
		print_timing_constraint_info("timing_constraints.echo");
	}

	/* Now normalize timing_constraint and constrained_ios to be in seconds, not nanoseconds. */
	for (source_clock_domain=0; source_clock_domain<num_constrained_clocks; source_clock_domain++) {
		for (sink_clock_domain=0; sink_clock_domain<num_constrained_clocks; sink_clock_domain++) {
			constraint = timing_constraint[source_clock_domain][sink_clock_domain];
			if (constraint > -0.01) { /* if constraint does not equal DO_NOT_ANALYSE */
				timing_constraint[source_clock_domain][sink_clock_domain] = constraint/1e9;
			}
		}
	}

	for (io_index = 0; io_index < num_constrained_ios; io_index++) {
		constrained_ios[io_index].delay /= 1e9;
	}

	/* Since all the information we need is stored in timing_constraint, constrained_clocks, and constrained_ios, free other data structures used in this routine */
	free(sdc_clocks);
	free(netlist_clocks);
	free(netlist_ios);
	return;
}

static void alloc_and_load_netlist_clocks_and_ios(void) {

	/* Count how many clocks and I/Os are in the netlist. 
	Store the names of each clock and each I/O in netlist_clocks and netlist_ios. */

	int iblock, i, net;
	char * name;
	boolean found;

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
			/* If we get here, the clock is new and so we add it to the array netlist_clocks */
				num_netlist_clocks++;
				/* dynamically grow the array to fit one new element */
				netlist_clocks = (char **) my_realloc (netlist_clocks, num_netlist_clocks * sizeof(t_clock));
				netlist_clocks[num_netlist_clocks - 1] = my_strdup(name);
			}
		} else if(logical_block[iblock].type == VPACK_INPAD || logical_block[iblock].type == VPACK_OUTPAD) {
			name = logical_block[iblock].name;
			/* Now that we've found an I/O, let's see if we've counted it already */
			found = FALSE;
			for (i = 0; !found && i < num_netlist_ios; i++) {
				if (strcmp(netlist_ios[i], name) == 0) {
					found = TRUE;
				}
			}
			if(!found) {
			/* If we get here, the I/O is new and so we add it to the array netlist_ios */
				num_netlist_ios++;
				/* dynamically grow the array to fit one new element */
				netlist_ios = (char **) my_realloc (netlist_ios, num_netlist_ios * sizeof(t_io));
				if (logical_block[iblock].type == VPACK_OUTPAD) {
					netlist_ios[num_netlist_ios - 1] = my_strdup(name + 4); 
					/* the + 4 removes the prefix "out:" automatically prepended to outputs */
				} else {
					netlist_ios[num_netlist_ios - 1] = my_strdup(name);
				}
			}
		}
	}
}

static void count_netlist_clocks_as_constrained_clocks(void) {
	/* Count how many clocks are in the netlist, and adds them to the array constrained_clocks. */

	int iblock, i, net;
	char * name;
	boolean found;

	num_constrained_clocks = 0;
	num_constrained_ios = 0;

	for (iblock = 0; iblock < num_logical_blocks; iblock++) {
		if(logical_block[iblock].type == VPACK_LATCH) {
			net = logical_block[iblock].clock_net;
			name = logical_block[net].name;
			/* Now that we've found a clock, let's see if we've counted it already */
			found = FALSE;
			for (i = 0; !found && i < num_constrained_clocks; i++) {
				if (strcmp(constrained_clocks[i].name, name) == 0) {
					found = TRUE;
				}
			}
			if(!found) {
			/* If we get here, the clock is new and so we add it to the array constrained_clocks */
				num_constrained_clocks++;
				/* dynamically grow the array to fit one new element */
				constrained_clocks = (t_clock *) my_realloc (constrained_clocks, num_constrained_clocks * sizeof(t_clock));
				constrained_clocks[num_constrained_clocks - 1].name = my_strdup(name);
				constrained_clocks[num_constrained_clocks - 1].is_netlist_clock = TRUE;
				/* Fanout will be filled out once the timing graph has been constructed. */
			}
		}
	}
}

static void get_sdc_tok(char * buf) {
/* Figures out which tokens are on this line and takes the appropriate actions. */

#define SDC_TOKENS " \t\n{}[]" 
	/* We're using so little of the SDC syntax that we can ignore braces */

	char * ptr, * from, * to, * clock_name;
	float clock_period, rising_edge, falling_edge, max_delay;
	int source_clock_domain, sink_clock_domain, iclock, iio, current_group_number = 0, num_exclusive_clocks = 0, num_multicycles;
	t_sdc_exclusive_group * exclusive_groups = NULL;
	boolean found;

	/* my_strtok splits the string into tokens - little character arrays separated by the SDC_TOKENS defined above.					   *
	 * Throughout this code, ptr refers to the tokens we fetch, one at a time.  The token changes at each call of my_strtok.		   *
	 * We call my_strtok with NULL as the first argument every time AFTER the first, since this picks up tokenizing where we left off. */
	ptr = my_strtok(buf, SDC_TOKENS, sdc, buf);

	if (!ptr) { /* blank line */
		return;
	}

	if (strcmp(ptr, "create_clock") == 0) {
		/* Syntax: create_clock -period <float> [-waveform {rising_edge falling_edge}] <clock names> */

		/* make sure clock has -period specified */
		
		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-period") != 0) {
			fprintf(stderr, "Create_clock must be followed by '-period' on line %d of SDC file.\n", file_line_number);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		/* Check if the token following -period is actually a number. */
		if(!check_if_number(ptr)) {
			fprintf(stderr, "Token following '-period' is not a number on line %d of SDC file.\n", file_line_number);
			exit(1);
		}
		
		clock_period = (float) strtod(ptr, NULL);

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(!ptr) {
			fprintf(stderr, "Clock net(s) not specified at end of line %d of SDC file.\n", file_line_number);
			exit(1);
		}
		
		if(strcmp(ptr, "-waveform") == 0) {
			/* Get the first float, which is the rising edge, and the second, which is the falling edge. */
			ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
			if(!check_if_number(ptr)) {
				fprintf(stderr, "First token following '-waveform' is not a number on line %d of SDC file.\n", file_line_number);
				exit(1);
			}
			rising_edge = (float) strtod(ptr, NULL);
			ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
			if(!check_if_number(ptr)) {
				fprintf(stderr, "Second token following '-waveform' is not a number on line %d of SDC file.\n", file_line_number);
				exit(1);
			}
			falling_edge = (float) strtod(ptr, NULL);
			/* Check that the falling edge is one half period away from the rising edge, excluding rounding error. */
			if(abs(rising_edge - falling_edge) - clock_period/2.0 > EQUAL_DEF) {
				fprintf(stderr, "Clock does not have 50%% duty cycle on line %d of SDC file.\n", file_line_number);
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

			/* We've found a new clock! */
			num_constrained_clocks++;

			/* Store the clock's name, period and offset in the local array sdc_clocks. */
			sdc_clocks = (t_sdc_clock *) my_realloc(sdc_clocks, num_constrained_clocks * sizeof(t_sdc_clock));
			sdc_clocks[num_constrained_clocks - 1].name = my_strdup(ptr);
			sdc_clocks[num_constrained_clocks - 1].period = clock_period;
			sdc_clocks[num_constrained_clocks - 1].offset = rising_edge; 
			
			/* Also store the clock's name, and the fact that it is not a netlist clock, in constrained_clocks. */
			constrained_clocks = (t_clock *) my_realloc (constrained_clocks, num_constrained_clocks * sizeof(t_clock));
			constrained_clocks[num_constrained_clocks - 1].name = my_strdup(ptr);
			constrained_clocks[num_constrained_clocks - 1].is_netlist_clock = FALSE;
			/* Fanout will be filled out once the timing graph has been constructed. */

			/* The next token should be NULL.  If so, return; if not, print an error message and exit. */
			ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
			if (!ptr) {
				return;
			}
			fprintf(stderr, "More than one virtual clock name is specified after -name on line %d of SDC file.\n", file_line_number);
			exit(1);

		} else {
			/* Parse through to the end of the line.  All that should be left on this line are one or more
			 * regular expressions denoting netlist clocks to be associated with this clock period.  An array sdc_clocks will
			 * store the period and offset of each clock at the same index which that clock has in netlist_clocks.  Later,
			 * after everything has been parsed, we take the information from this array to calculate the actual timing constraints
			 * which these periods and offsets imply, and put them in the matrix timing_constraint. */

			for(;;) {
				if(!ptr) { /* end of line */
					return; 
				}

				found = FALSE;

				for (iclock = 0; iclock < num_netlist_clocks; iclock++) {
					/* See if the regular expression stored in ptr is legal and matches at least one clock net. 
					If it is not legal, it will fail during regex_match.  We check for a match using boolean found. */
					if (regex_match(netlist_clocks[iclock], ptr)) {
						/* We've found a new clock! */
						num_constrained_clocks++;
						found = TRUE;

						/* Store the clock's name, period and offset in the local array sdc_clocks. */
						sdc_clocks = (t_sdc_clock *) my_realloc(sdc_clocks, num_constrained_clocks * sizeof(t_sdc_clock));
						sdc_clocks[num_constrained_clocks - 1].name = my_strdup(netlist_clocks[iclock]);
						sdc_clocks[num_constrained_clocks - 1].period = clock_period;
						sdc_clocks[num_constrained_clocks - 1].offset = rising_edge; 

						/* Also store the clock's name, and the fact that it is a netlist clock, in constrained_clocks. */
						constrained_clocks = (t_clock *) my_realloc (constrained_clocks, num_constrained_clocks * sizeof(t_clock));
						constrained_clocks[num_constrained_clocks - 1].name = my_strdup(netlist_clocks[iclock]);
						constrained_clocks[num_constrained_clocks - 1].is_netlist_clock = TRUE;
						/* Fanout will be filled out once the timing graph has been constructed. */
					}
				}

				if (!found) {
					fprintf(stderr, "Clock name or regular expression %s on line %d of SDC file does not correspond to any nets.\n" 
						"If you'd like to create a virtual clock, use the -name keyword.\n", ptr, file_line_number);
					exit(1);
				}

				/* Advance to the next token (or the end of the line). */
				ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
			}	
		}
	} else if (strcmp(ptr, "set_clock_groups") == 0) {
		/* Syntax: set_clock_groups -exclusive -group {<clock name(s)>} -group {<clock name(s)>} [-group {<clock name(s)>} ...] */

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-exclusive") != 0) {
			fprintf(stderr, "Set_clock_groups must be followed by '-exclusive' on line %d of SDC file.\n", file_line_number);
			exit(1);
		}
		
		/* Parse through to the end of the line.  All that should be left on this line are a bunch of 
		 * -group commands, followed by lists of clocks in that group.  An array exclusive_clock_groups will
		 * store the temporary "group number" of each clock along with its name.  We then add an entry to override_constraints for 
		 * each pair of clocks with different group numbers, so we do not analyse between clock domains in different groups. */

		for(;;) {
			ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
			if(!ptr) { /* end of line */
				break; /* exit the infinite for loop - but don't return yet - we still have to populate override_constraints!  */
			}
			if(strcmp(ptr, "-group") == 0) {
			/* add 1 to the group number we're assigning clocks to every time the token -group is hit */
				current_group_number++;
			} else { 
				/* We have a clock name */
				num_exclusive_clocks++;
				exclusive_groups = (t_sdc_exclusive_group *) my_realloc(exclusive_groups, num_exclusive_clocks * sizeof(t_sdc_exclusive_group));
				exclusive_groups[num_exclusive_clocks - 1].name = my_strdup(ptr);
				exclusive_groups[num_exclusive_clocks - 1].group = current_group_number;
			}
		}

		/* Finally, create an override constraint for each pair of source and sink clock domains in exclusive_groups which have different group numbers,
		so that we DO_NOT_ANALYSE them */
		for(source_clock_domain=0; source_clock_domain<num_exclusive_clocks; source_clock_domain++) {
			for(sink_clock_domain=0; sink_clock_domain<num_exclusive_clocks; sink_clock_domain++) {
				if(exclusive_groups[source_clock_domain].group != exclusive_groups[sink_clock_domain].group) {
					num_override_constraints++;
					override_constraints = (t_sdc_override_constraint *) my_realloc(override_constraints, num_override_constraints * sizeof(t_sdc_override_constraint));
					override_constraints[num_override_constraints - 1].source_clock_domain = my_strdup(exclusive_groups[source_clock_domain].name);
					override_constraints[num_override_constraints - 1].sink_clock_domain = my_strdup(exclusive_groups[sink_clock_domain].name);
					override_constraints[num_override_constraints - 1].constraint = DO_NOT_ANALYSE;
					override_constraints[num_override_constraints - 1].num_multicycles = 0.; /* no multicycling */
				}
			}
		}

		free(exclusive_groups);
		return;

	} else if (strcmp(ptr, "set_false_path") == 0) {
		/* Syntax: set_false_path -from <clock name> -to <clock name> */

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-from") != 0) {
			fprintf(stderr, "Set_false_path must be followed by '-from <clock_name>' on line %d of SDC file.\n", file_line_number);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		from = ptr;

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-to") != 0) {
			fprintf(stderr, "Set_false_path must be followed by '-from <clock_name> -to <clock_name> on line %d of SDC file.\n", file_line_number);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		to = ptr;

		/* Create an override constraint between from and to with value DO_NOT_ANALYSE. */
		num_override_constraints++;
		override_constraints = (t_sdc_override_constraint *) my_realloc(override_constraints, num_override_constraints * sizeof(t_sdc_override_constraint));
		override_constraints[num_override_constraints - 1].source_clock_domain = my_strdup(from);
		override_constraints[num_override_constraints - 1].sink_clock_domain = my_strdup(to);
		override_constraints[num_override_constraints - 1].constraint = DO_NOT_ANALYSE;
		override_constraints[num_override_constraints - 1].num_multicycles = 0.; /* no multicycling */
		
		return;

	} else if (strcmp(ptr, "set_max_delay") == 0) {
		/* Syntax: set_max_delay <delay> -from <clock name> -to <clock name> */

		/* Basically the same as set_false_path above, except we get a specific delay value for the override constraint. */

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		/* check if the token following set_max_delay is actually a number*/
		if(!check_if_number(ptr)) {
			fprintf(stderr, "Token following 'set_max_delay' is not a number on line %d of SDC file.\n", file_line_number);
			exit(1);
		}
		max_delay = (float) strtod(ptr, NULL);

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-from") != 0) {
			fprintf(stderr, "Set_false_path must be followed by '-from <clock_name>' on line %d of SDC file.\n", file_line_number);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		from = ptr;

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-to") != 0) {
			fprintf(stderr, "Set_false_path must be followed by '-from <clock_name> -to <clock_name> on line %d of SDC file.\n", file_line_number);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		to = ptr;

		/* Create an override constraint between from and to with value max_delay. */
		num_override_constraints++;
		override_constraints = (t_sdc_override_constraint *) my_realloc(override_constraints, num_override_constraints * sizeof(t_sdc_override_constraint));
		override_constraints[num_override_constraints - 1].source_clock_domain = my_strdup(from);
		override_constraints[num_override_constraints - 1].sink_clock_domain = my_strdup(to);
		override_constraints[num_override_constraints - 1].constraint = max_delay;
		override_constraints[num_override_constraints - 1].num_multicycles = 0.; /* no multicycling */

		return;

	} else if (strcmp(ptr, "set_multicycle_path") == 0) {
		/* Syntax: set_multicycle_path -from <clock name> -to <clock name> <num_multicycles>*/

		/* Basically the same as set_false_path and set_max_delay above, except we have to calculate the default value of the constraint (obtained via edge counting)
		first, and then set an override constraint equal to default constraint + (num_multicycles - 1) * period of sink clock domain. */

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		/* check if the token following set_max_delay is actually a number*/
		if(strcmp(ptr, "-setup") !=0) {
			fprintf(stderr, "Set_multicycle_path must be followed by '-setup' on line %d of SDC file.\n", file_line_number);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-from") != 0) {
			fprintf(stderr, "Set_multicycle_path must be followed by '-from <clock_name>' on line %d of SDC file.\n", file_line_number);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		from = ptr;

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-to") != 0) {
			fprintf(stderr, "Set_multicycle_path must be followed by '-from <clock_name> -to <clock_name> on line %d of SDC file.\n", file_line_number);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		to = ptr;

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		/* check if the token following set_max_delay is actually a number*/
		if(!check_if_number(ptr)) {
			fprintf(stderr, "Token following '-to <clock_name>' is not a number on line %d of SDC file.\n", file_line_number);
			exit(1);
		}
		num_multicycles = (float) strtod(ptr, NULL);

		/* Create an override constraint between from and to. Unlike the previous two commands, set_multicycle_path requires information about the periods and offsets 
		of the two clocks, which we have to fill in at the end. */
		num_override_constraints++;
		override_constraints = (t_sdc_override_constraint *) my_realloc(override_constraints, num_override_constraints * sizeof(t_sdc_override_constraint));
		override_constraints[num_override_constraints - 1].source_clock_domain = my_strdup(from);
		override_constraints[num_override_constraints - 1].sink_clock_domain = my_strdup(to);
		override_constraints[num_override_constraints - 1].num_multicycles = num_multicycles;

		return;

	} if (strcmp(ptr, "set_input_delay") == 0) {
		/* Syntax: set_input_delay -clock <virtual or netlist clock> -max <max_delay> [get_ports {<port_list>}] */
		
		/* We want to assign virtual_clock to all input ports in port_list, and set the input delay (from the external device to the FPGA) to max_delay. */

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-clock") != 0) {
			fprintf(stderr, "Set_input_delay must be followed by '-clock <virtual or netlist clock name>' on line %d of SDC file.\n", file_line_number);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		clock_name = ptr;
	
		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-max") != 0) {
			fprintf(stderr, "Set_input_delay -clock <virtual or netlist clock name> must be followed by '-max <maximum_input_delay>' on line %d of SDC file.\n", file_line_number);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		/* check if the token following -max is actually a number*/
		if(!check_if_number(ptr)) {
			fprintf(stderr, "Token following '-max' is not a number on line %d of SDC file.\n", file_line_number);
			exit(1);
		}
		max_delay = (float) strtod(ptr, NULL);

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "get_ports") != 0) {
			fprintf(stderr, "Set_input_delay requires a [get_ports {...}] command following '-max <max_input_delay>' on line %d of SDC file.\n", file_line_number);
			exit(1);
		}

		/* Parse through to the end of the line.  Add each regular expression match we find to the list of 
			constrained I/Os and give each entry the virtual clock name and max_delay we've just parsed.  
			We have no way of error-checking whether these tokens correspond to actual I/O ports until later. */
		
		for(;;) {
			ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
			if(!ptr) { /* end of line */
				return; 
			}

			found = FALSE;

			for (iio = 0; iio < num_netlist_ios; iio++) {
				/* See if the regular expression stored in ptr is legal and matches at least one I/O port. 
				If it is not legal, it will fail during regex_match.  We check for a match using boolean found. */
				if (regex_match(netlist_ios[iio], ptr)) {
					/* We've found a new I/O! */
					num_constrained_ios++;
					found = TRUE;

					/* Fill in I/O information in the permanent array constrained_ios. */
					constrained_ios = (t_io *) my_realloc (constrained_ios, num_constrained_ios * sizeof(t_io));
					constrained_ios[num_constrained_ios - 1].name = my_strdup(ptr);
					constrained_ios[num_constrained_ios - 1].virtual_clock_name = my_strdup(clock_name);
					constrained_ios[num_constrained_ios - 1].delay = max_delay;
				}
			}

			if (!found) {
				fprintf(stderr, "I/O name or regular expression %s on" 
					"line %d of SDC file does not correspond to any nets.\n", ptr, file_line_number);
				exit(1);
			}
		}

	} else if (strcmp(ptr, "set_output_delay") == 0) {
		/* Syntax: set_output_delay -clock <virtual or netlist clock> -max <max_delay> [get_ports {<port_list>}] */
		
		/* We want to assign virtual_clock to all output ports in port_list, and set the output delay (from the external device to the FPGA) to max_delay. */

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-clock") != 0) {
			fprintf(stderr, "Set_output_delay must be followed by '-clock <virtual_clock_name>' on line %d of SDC file.\n", file_line_number);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		clock_name = ptr;
		/* We have no way of error-checking whether this is an actual virtual clock until we finish parsing. */

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "-max") != 0) {
			fprintf(stderr, "Set_output_delay -clock <virtual or netlist clock name> must be followed by '-max <maximum_output_delay>' on line %d of SDC file.\n", file_line_number);
			exit(1);
		}

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		/* check if the token following -max is actually a number*/
		if(!check_if_number(ptr)) {
			fprintf(stderr, "Token following '-max' is not a number on line %d of SDC file.\n", file_line_number);
			exit(1);
		}
		max_delay = (float) strtod(ptr, NULL);

		ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
		if(strcmp(ptr, "get_ports") != 0) {
			fprintf(stderr, "Set_output_delay requires a [get_ports {...}] command following '-max <max_output_delay>' on line %d of SDC file.\n", file_line_number);
			exit(1);
		}

		/* Parse through to the end of the line.  Add each regular expression match we find to the list of 
			constrained I/Os and give each entry the virtual clock name and max_delay we've just parsed.  
			We have no way of error-checking whether these tokens correspond to actual I/O ports until later. */
		
		for(;;) {
			ptr = my_strtok(NULL, SDC_TOKENS, sdc, buf);
			if(!ptr) { /* end of line */
				return; 
			}

			found = FALSE;

			for (iio = 0; iio < num_netlist_ios; iio++) {
				/* See if the regular expression stored in ptr is legal and matches at least one I/O port. 
				If it is not legal, it will fail during regex_match.  We check for a match using boolean found. */
				if (regex_match(netlist_ios[iio], ptr)) {
					/* We've found a new I/O! */
					num_constrained_ios++;
					found = TRUE;

					/* Fill in I/O information in the permanent array constrained_ios. */
					constrained_ios = (t_io *) my_realloc (constrained_ios, num_constrained_ios * sizeof(t_io));
					constrained_ios[num_constrained_ios - 1].name = my_strdup(ptr);
					constrained_ios[num_constrained_ios - 1].virtual_clock_name = my_strdup(clock_name);
					constrained_ios[num_constrained_ios - 1].delay = max_delay;
				}
			}

			if (!found) {
				fprintf(stderr, "I/O name or regular expression %s on" 
					"line %d of SDC file does not correspond to any nets.\n", ptr, file_line_number);
				exit(1);
			}
		}

	} else {
		fprintf(stderr, "Incorrect or unsupported syntax near start of line %d of SDC file.\n", file_line_number);
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

static int find_constrained_clock(char * ptr) {
/* Given a string ptr, find whether it's the name of a clock in the array constrained_clocks.  *
 * if it is, return the clock's index in constrained_clocks; if it's not, return -1. */
	int index;
	for(index=0; index<num_constrained_clocks; index++) {
		if(strcmp(ptr, constrained_clocks[index].name) == 0) {
			return index;
		}
	}
	return -1;
}

static int find_override_constraint(int source_clock_domain, int sink_clock_domain) {
	/* Given a pair of source and sink clock domains, find out if there's an override constraint between them.
	If there is, return the index in override_constraints; if there is not, return -1. */
	int index;
	char * source_clock_name, * sink_clock_name;

	source_clock_name = constrained_clocks[source_clock_domain].name;
	sink_clock_name = constrained_clocks[sink_clock_domain].name;

	for(index=0; index<num_override_constraints; index++) {
		if(strcmp(source_clock_name, override_constraints[index].source_clock_domain) == 0 
			&& strcmp(sink_clock_name, override_constraints[index].sink_clock_domain) == 0) {
			return index;
		}
	}
	return -1;
}

static float calculate_constraint(t_sdc_clock source_domain, t_sdc_clock sink_domain) {
	/* Given information from the SDC file about the period and offset of two clocks, *
	 * determine the implied setup-time constraint between them via edge counting.    */

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

static void print_timing_constraint_info(char *fname) {
	/* Prints the contents of timing_constraint, constrained_clocks, constrained_ios and override_constraints to a file. */
	
	FILE * fp;
	int source_clock_domain, sink_clock_domain, i;
	int * clock_name_length = (int *) my_malloc(num_constrained_clocks * sizeof(int)); /* Array of clock name lengths */
	int max_clock_name_length = INT_MIN;
	char * clock_name;

	fp = my_fopen(fname, "w", 0);

	/* Get lengths of each clock name and max length so we can space the columns properly. */
	for (sink_clock_domain = 0; sink_clock_domain < num_constrained_clocks; sink_clock_domain++) {
		clock_name = constrained_clocks[sink_clock_domain].name;
		clock_name_length[sink_clock_domain] = strlen(clock_name);
		if (clock_name_length[sink_clock_domain] > max_clock_name_length) {
			max_clock_name_length = clock_name_length[sink_clock_domain];	
		}
	}

	/* First, combine info from timing_constraint and constrained_clocks into a matrix 
	(they're indexed the same as each other). */

	fprintf(fp, "Timing constraints in ns (source clock domains down left side, sink along top).\n");
	fprintf(fp, "A value of -1.00 means the pair of source and sink domains will not be analysed.\n\n");

	print_spaces(fp, max_clock_name_length + 4); 

	for (sink_clock_domain = 0; sink_clock_domain < num_constrained_clocks; sink_clock_domain++) {
		fprintf(fp, "%s", constrained_clocks[sink_clock_domain].name);
		/* Minimum column width of 8 */
		print_spaces(fp, max(8 - clock_name_length[sink_clock_domain], 4));
	}
	fprintf(fp, "\n");

	for (source_clock_domain = 0; source_clock_domain < num_constrained_clocks; source_clock_domain++) {
		fprintf(fp, "%s", constrained_clocks[source_clock_domain].name);
		print_spaces(fp, max_clock_name_length + 4 - clock_name_length[source_clock_domain]);
		for (sink_clock_domain = 0; sink_clock_domain < num_constrained_clocks; sink_clock_domain++) {
			fprintf(fp, "%5.2f", timing_constraint[source_clock_domain][sink_clock_domain]);
			/* Minimum column width of 8 */
			print_spaces(fp, max(clock_name_length[sink_clock_domain] - 1, 3));
		}
		fprintf(fp, "\n");
	}

	/* Second, print I/O constraints. */
	fprintf(fp, "\nList of constrained I/Os:\n");
	for(i = 0; i < num_constrained_ios; i++) {
		fprintf(fp, "I/O name %s on clock %s with input/output delay %.2f ns\n", 
			constrained_ios[i].name, constrained_ios[i].virtual_clock_name, constrained_ios[i].delay);
	}

	/* Third, print override constraints. */
	fprintf(fp, "\nList of override constraints (non-default constraints created by set_false_path, set_clock_groups, \nset_max_delay, and set_multicycle_path):\n");
	for(i = 0; i < num_override_constraints; i++) {
		if (override_constraints[i].num_multicycles == 0) { /* not a multicycle constraint */
			fprintf(fp, "%s to %s: %.2f ns\n", 
				override_constraints[i].source_clock_domain, override_constraints[i].sink_clock_domain, override_constraints[i].constraint);
		} else { /* multicycle constraint */
			fprintf (fp, "%s to %s: %d multicycles\n", 
				override_constraints[i].source_clock_domain, override_constraints[i].sink_clock_domain, override_constraints[i].num_multicycles);
		}
	}

	fclose(fp);
	free(clock_name_length);
}

static void print_spaces(FILE * fp, int num_spaces) {
	/* Prints num_spaces spaces to file pointed to by fp. */
	for( ; num_spaces > 0; num_spaces--) {
		fprintf(fp, " ");
	}
}

static boolean regex_match (char *string, char *pattern) {
	/* Given a string and a regular expression pattern, 
	return TRUE if there's a match, FALSE if not. Print
	an error and exit if something is wrong with pattern. */

    int status, rc;
    regex_t regex; /* The "compiled" regular expression. */
	char buffer[100];
	
	/* Special case for SDC compatibility: if pattern is "*", always match. */
	if (strcmp(pattern, "*") == 0) {
		return TRUE;
	}

	/* Otherwise, compile a regular expression from pattern.  If it's valid, we should get rc = 0. */
	rc = regcomp(&regex, pattern, 0); 
    if (rc != 0) { 
		/* Put the error code rc and the address &regex of the compiled regex into regerror. 
		Regerror spits out the appropriate error message into a buffer of size 100. */
        regerror(rc, &regex, buffer, 100);
		/* Now print the error message stored in buffer. */
		fprintf(stderr, "Regular expression %s on line %d could not be compiled because of: %s", string, file_line_number, buffer);
		exit(1);
    }

	/* Execute the regular expression comparison between the compiled regex and string. */
    status = regexec(&regex, string, 0, NULL, 0);

	/* Free up regex_t regex. */
    regfree(&regex);

	/* Based on status, either return TRUE (if a match), FALSE (if not), or an error. */
    if (status == 0) {
        return TRUE;      
    } else if (status == REG_NOMATCH) {
		return FALSE;
	} else {
		/* Put the error code status and the address &regex of the compiled regex into regerror. 
		Regerror spits out the appropriate error message into a buffer of size 100. */
        regerror(status, &regex, buffer, 100);
		/* Now print the error message stored in buffer. */
		fprintf(stderr, "Regular expression %s on line %d could not be executed because of: %s", string, file_line_number, buffer);
		exit(1);
	}
}