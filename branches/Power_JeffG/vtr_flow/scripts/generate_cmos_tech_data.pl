#!/usr/bin/perl

use strict;
use POSIX;
use File::Basename;

sub get_trans_properties;
sub get_optimal_sizing;
sub get_riset_fallt_diff;
sub get_Vth;
sub add_subckts;

my $hspice = "hspice";

my $number_arguments = @ARGV;
if ( $number_arguments < 4 ) {
	print("usage: generate_cmos_tech_data.pl <tech_file> <tech_size> <Vdd> <temp>\n");
	exit(-1);
}

my $tech_file      = shift(@ARGV);
my $tech_size      = shift(@ARGV);
my $Vdd            = shift(@ARGV);
my $temp           = shift(@ARGV);
my $tech_file_name = basename($tech_file);

-r $tech_file or die "Cannot open tech file ($tech_file)";

my $spice_file = "vtr_auto_trans_info.sp";
my $spice_out  = "vtr_auto_trans_info.out";

my $leakage_high;
my $leakage_low;
my $C_gate_cmos;
my $C_source_cmos;
my $C_drain_cmos;
my $C_gate_pass;
my $C_source_pass;
my $C_drain_pass;

my $full = 1;

my @sizes;
my @long_size        = ( 1,      2 );
my @transistor_types = ( "nmos", "pmos" );

if ( !$full ) {
	my @transistor_types = ("pmos");
}

my $optimal_p_to_n;
if ($full) {
	$optimal_p_to_n = get_optimal_sizing();
}
else {
	$optimal_p_to_n = 1.0;
}

if ($full) {
	my $size = 1.0;
	while ( $size < 1000 ) {
		my $size_rounded = ( sprintf "%.2f", $size );
		push( @sizes, [ $size_rounded, 1 ] );
		$size = $size * 1.02;
	}
}
else {
	push( @sizes, [ 1, 1 ] );
}

print "<technology file=\"$tech_file_name\" size=\"$tech_size\">\n";

print "\t<operating_point temperature=\"$temp\" Vdd=\"$Vdd\"/>\n";
print "\t<p_to_n ratio=\"" . $optimal_p_to_n . "\"/>\n";

foreach my $type (@transistor_types) {
	my $Vth = get_Vth();

	print "\t<transistor type=\"$type\" Vth=\"$Vth\">\n";

	( $leakage_high, $leakage_low, $C_gate_cmos, $C_drain_cmos, $C_gate_pass, $C_source_pass, $C_drain_pass ) = get_trans_properties( $type, $long_size[0], $long_size[1], $Vth );
	print "\t\t<long_size W=\"" . $long_size[0] . "\" L=\"" . $long_size[1] . "\">\n";
	print "\t\t\t<leakage_current high=\"$leakage_high\" low=\"$leakage_low\"/>\n";
	print "\t\t\t<capacitance gate_cmos=\"$C_gate_cmos\" drain_cmos=\"$C_drain_cmos\" gate_pass=\"$C_gate_pass\" source_pass=\"$C_source_pass\" drain_pass=\"$C_drain_pass\"/>\n";
	print "\t\t</long_size>\n";

	foreach my $size_ref (@sizes) {
		my @size = @$size_ref;
		( $leakage_high, $leakage_low, $C_gate_cmos, $C_drain_cmos, $C_gate_pass, $C_source_pass, $C_drain_pass ) = get_trans_properties( $type, $size[0], $size[1], $Vth );
		print "\t\t<size W=\"" . $size[0] . "\" L=\"" . $size[1] . "\">\n";
		print "\t\t\t<leakage_current high=\"$leakage_high\" low=\"$leakage_low\"/>\n";
		print "\t\t\t<capacitance gate_cmos=\"$C_gate_cmos\" drain_cmos=\"$C_drain_cmos\" gate_pass=\"$C_gate_pass\" source_pass=\"$C_source_pass\" drain_pass=\"$C_drain_pass\"/>\n";
		print "\t\t</size>\n";
	}
	print "\t</transistor>\n";
}

print "</technology>\n";

sub get_optimal_sizing {
	my @sizes;
	my $opt_size;
	my $opt_percent;

	for ( my $size = 0.5 ; $size < 4 ; $size += 0.05 ) {
		push( @sizes, $size );
	}

	$opt_percent = 10000;    # large number
	foreach my $size (@sizes) {
		my $diff = get_riset_fallt_diff($size);
		if ( $diff < $opt_percent ) {
			$opt_size    = $size;
			$opt_percent = $diff;
		}
	}

	return $opt_size;
}

sub add_subckts {
	my $s = shift(@_);

	$s = $s . "\n";
	$s = $s . ".subckt nfet drain gate source body size=1\n";
	$s = $s . "M1 drain gate source body nmos L='tech' W='size*tech' AS='size*tech*2.5*tech' PS='2*2.5*tech+size*tech' AD='size*tech*2.5*tech' PD='2*2.5*tech+size*tech'\n";
	$s = $s . ".ends\n";
	$s = $s . ".subckt pfet drain gate source body size=1\n";
	$s = $s . "M1 drain gate source body pmos L='tech' W='size*tech' AS='size*tech*2.5*tech' PS='2*2.5*tech+size*tech' AD='size*tech*2.5*tech' PD='2*2.5*tech+size*tech'\n";
	$s = $s . ".ends\n";
	$s = $s . ".subckt pfetz drain gate source body wsize=1 lsize=1\n";
	$s = $s . "M1 drain gate source body pmos L='lsize*tech' W='wsize*tech' AS='wsize*tech*2.5*lsize*tech' PS='2*2.5*lsize*tech+wsize*tech' AD='wsize*tech*2.5*lsize*tech' PD='2*2.5*lsize*tech+wsize*tech'\n";
	$s = $s . ".ends\n";
	$s = $s . ".subckt inv in out Vdd Gnd nsize=1 psize=2\n";
	$s = $s . "X0 out in Gnd Gnd nfet size='nsize'\n";
	$s = $s . "X1 out in Vdd Vdd pfet size='psize'\n";
	$s = $s . ".ends\n";
	$s = $s . ".subckt levr in out Vdd Gnd\n";
	$s = $s . "X0 in out Vdd Gnd inv nsize=2 psize=1\n";
	$s = $s . "X1 in out Vdd Vdd pfetz wsize=1 lsize=2\n";
	$s = $s . ".ends\n";
	$s = $s . "\n";

	return $s;
}

sub get_Vth {
	my $type = shift(@_);

	my $s = "";
	my $Vth;

	$s = $s . "HSpice Simulation of a $type transistor.\n";
	$s = $s . ".include \"$tech_file\"\n";
	$s = $s . ".param tech = $tech_size\n";
	$s = $s . ".param Vol = $Vdd\n";
	$s = $s . ".param size = 1.0\n";
	$s = $s . ".param simt = 10u\n";

	$s = add_subckts($s);

	# Add voltage source
	$s = $s . "Vdd Vdd 0 Vol\n";
	$s = $s . "Vin Vin 0 Vol\n";

	# Transistors
	if ( $type eq "nmos" ) {

		$s = $s . "X0 Vin Vdd out 0 nfet size='size'\n";
		$s = $s . "X1a 0 0 out 0 nfet size='size'\n";
		$s = $s . "X1b 0 0 out 0 nfet size='size'\n";
		$s = $s . "X1c 0 0 out 0 nfet size='size'\n";
		$s = $s . "X1d 0 0 out 0 nfet size='size'\n";
	}
	else {
		$s = $s . "X0 0 0 out Vdd pfet size='size'\n";
		$s = $s . "X1a Vin Vdd out Vdd pfet size='size'\n";
		$s = $s . "X1b Vin Vdd out Vdd pfet size='size'\n";
		$s = $s . "X1c Vin Vdd out Vdd pfet size='size'\n";
		$s = $s . "X1d Vin Vdd out Vdd pfet size='size'\n";
	}

	$s = $s . ".TEMP $temp\n";
	$s = $s . ".OP\n";
	$s = $s . ".OPTIONS LIST NODE POST\n";

	$s = $s . ".tran 'simt/10000' 'simt'\n";
	if ( $type eq "nmos" ) {
		$s = $s . ".measure tran vth avg PAR('Vol - V(out)')\n";
	}
	else {
		$s = $s . ".measure tran vth avg PAR('V(out)')\n";
	}
	$s = $s . ".end\n\n";

	open( SP_FILE, "> $spice_file" );
	print SP_FILE $s;
	close(SP_FILE);

	system("$hspice $spice_file 1> $spice_out 2> /dev/null");

	my $spice_data;
	{
		local $/ = undef;
		open( SP_FILE, "$spice_out" );
		$spice_data = <SP_FILE>;
		close SP_FILE;
	}

	if ( $spice_data =~ /transient analysis.*?vth\s*=\s*(\S+)/s ) {
		$Vth = $1;
	}
	else {
		die "Could not find threshold voltage in spice output.\n";
	}

	return $Vth;
}

sub get_riset_fallt_diff {
	my $size = shift(@_);

	my $s = "";
	my $rise;
	my $fall;

	$s = $s . "HSpice Simulation of an inverter.\n";
	$s = $s . ".include \"$tech_file\"\n";
	$s = $s . ".param tech = $tech_size\n";
	$s = $s . ".param Vol = $Vdd\n";
	$s = $s . ".param psize = '2*$size'\n";
	$s = $s . ".param nsize = '2.0'\n";
	$s = $s . ".param Wn = 'nsize*tech'\n";
	$s = $s . ".param Wp = 'psize*tech'\n";
	$s = $s . ".param L = 'tech'\n";
	$s = $s . ".param D = '2.5*tech'\n";

	# Add voltage source
	$s = $s . "Vdd Vdd 0 Vol\n";

	# Input Voltage
	$s = $s . "Vin gate 0 PULSE(0 Vol 10n 0.01n 0.01n 10n 20n)\n";

	# Transistors
	$s = $s . "M1 out gate 0 0 nmos L='L' W='Wn' AS='Wn*D' PS='2*D+Wn' AD='Wn*D' PD='2*D+Wn'\n";
	$s = $s . "M2 out gate Vdd Vdd pmos L='L' W='Wp' AS='Wp*D' PS='2*D+Wp' AD='Wp*D' PD='2*D+Wp'\n";

	$s = $s . ".TEMP $temp\n";
	$s = $s . ".OP\n";
	$s = $s . ".OPTIONS LIST NODE POST\n";

	$s = $s . ".tran 1f 30n\n";

	$s = $s . ".measure tran fallt TRIG V(gate) VAL = '0.5*Vol' TD = 9ns RISE = 1 TARG V(out) VAL = '0.5*Vol' FALL = 1\n";
	$s = $s . ".measure tran riset TRIG V(gate) VAL = '0.5*Vol' TD = 19ns FALL = 1 TARG V(out) VAL = '0.5*Vol' RISE = 1\n";
	$s = $s . ".end\n\n";

	open( SP_FILE, "> $spice_file" );
	print SP_FILE $s;
	close(SP_FILE);

	system("$hspice $spice_file 1> $spice_out 2> /dev/null");

	my $spice_data;
	{
		local $/ = undef;
		open( SP_FILE, "$spice_out" );
		$spice_data = <SP_FILE>;
		close SP_FILE;
	}

	if ( $spice_data =~ /transient analysis.*?riset\s*=\s*(\S+)/s ) {
		$rise = $1;
	}
	else {
		die "Could not find rise time in spice output.\n";
	}

	if ( $spice_data =~ /transient analysis.*?fallt\s*=\s*(\S+)/s ) {
		$fall = $1;
	}
	else {
		die "Could not find fall time in spice output.\n";
	}

	return abs( $rise - $fall ) / $rise;
}

sub get_trans_properties {
	my $type   = shift(@_);
	my $width  = shift(@_);
	my $length = shift(@_);
	my $Vth    = shift(@_);

	my $s = "";
	my $leakage;
	my $C_gate;
	my $C_source;
	my $C_drain;
	my $sim_time = "600u";

	$s = $s . "HSpice Simulation of a $type transistor.\n";
	$s = $s . ".include \"$tech_file\"\n";
	$s = $s . ".param tech = $tech_size\n";
	$s = $s . ".param Vol = $Vdd\n";
	$s = $s . ".param Vdt = 'Vol - $Vth'\n";
	$s = $s . ".param Vth = $Vth\n";
	$s = $s . ".param W = '" . $width . "*tech'\n";
	$s = $s . ".param L = '" . $length . "*tech'\n";
	$s = $s . ".param D = '2.5*tech'\n";
	$s = $s . ".param simt = $sim_time\n";
	$s = $s . ".param tick = 'simt/8'\n";	
	$s = $s . ".param rise = 'tick/100'\n";

	# Gate, Drain, Source Inputs
	# Time	NMOS		PMOS	Valid For:
	# 		G 	D 	S		G 	D 	S
	# 0		0 	0 	0		0 	0 	0	CMOS, Pass-Logic
	# 1		0 	0 	1     	0 	1 	1	CMOS, Pass-Logic, High Leakage
	# 2		0 	1 	0     	1 	0 	0	Pass-Logic
	# 3		0 	1 	1     	1 	0 	1	Pass-Logic
	# 4		1 	0 	0     	1 	1 	0	Pass-Logic
	# 5		1 	1 	1	    1 	1 	1	Pass-Logic
	# 6     0 	d-t 0		1 	1 	t   Low-Leakage
	# 7 	0 	0 	d-t		1 	t 	1 	Low-Leakage

	$s = $s . ".param t0s = '0*tick+2*rise'\n";
	$s = $s . ".param t0e = '1*tick-rise'\n";
	$s = $s . ".param t1s = '1*tick+2*rise'\n";
	$s = $s . ".param t1e = '2*tick-rise'\n";
	$s = $s . ".param t2s = '2*tick+2*rise'\n";
	$s = $s . ".param t2e = '3*tick-rise'\n";
	$s = $s . ".param t3s = '3*tick+2*rise'\n";
	$s = $s . ".param t3e = '4*tick-rise'\n";
	$s = $s . ".param t4s = '4*tick+2*rise'\n";
	$s = $s . ".param t4e = '5*tick-rise'\n";
	$s = $s . ".param t5s = '5*tick+2*rise'\n";
	$s = $s . ".param t5e = '6*tick-rise'\n";
	$s = $s . ".param t6s = '6*tick+2*rise'\n";
	$s = $s . ".param t6e = '7*tick-rise'\n";
	$s = $s . ".param t7s = '7*tick+2*rise'\n";
	$s = $s . ".param t7e = '8*tick-rise'\n";

	$s = $s . "Vdd Vdd 0 Vol\n";

	if ( $type eq "nmos" ) {
		$s = $s . "Vgate gate 0 PWL(	0 0 	'4*tick' 0 '4*tick+rise' Vol 	'6*tick' Vol '6*tick+rise' 0)\n";
		$s = $s . "Vdrain drain 0 PWL(	0 0 	'2*tick' 0 '2*tick+rise' Vol 	'4*tick' Vol '4*tick+rise' 0 	'5*tick' 0 '5*tick+rise' Vol 	'6*tick' Vol '6*tick+rise' Vdt 	'7*tick' Vdt '7*tick+rise' 0)\n";
		$s = $s . "Vsource source 0 PWL(0 0 	'1*tick' 0 '1*tick+rise' Vol 	'2*tick' Vol '2*tick+rise' 0 	'3*tick' 0 '3*tick+rise' Vol 	'4*tick' Vol '4*tick+rise' 0 	'5*tick' 0 '5*tick+rise' Vol 	'6*tick' Vol '6*tick+rise' 0 	'7*tick' 0 '7*tick+rise' Vdt)\n";

		$s = $s . "M1 drain gate source 0 $type L='L' W='W' AS='W*D' PS='2*D+W' AD='W*D' PD='2*D+W'\n";
	}
	else {
		$s = $s . "Vgate gate 0 PWL(	0 0 	'2*tick' 0 '2*tick+rise' Vol)\n";
		$s = $s . "Vdrain drain 0 PWL(	0 0 	'1*tick' 0 '1*tick+rise' Vol 	'2*tick' Vol '2*tick+rise' 0 	'4*tick' 0 '4*tick+rise' Vol 	'7*tick' Vol '7*tick+rise' Vth)\n";
		$s = $s . "Vsource source 0 PWL(0 0 	'1*tick' 0 '1*tick+rise' Vol 	'2*tick' Vol '2*tick+rise' 0 	'3*tick' 0 '3*tick+rise' Vol 	'4*tick' Vol '4*tick+rise' 0 	'5*tick' 0 '5*tick+rise' Vol	'6*tick' Vol '6*tick+rise' Vth	'7*tick' Vth '7*tick+rise' Vol)\n";

		$s = $s . "M1 drain gate source Vdd $type L='L' W='W' AS='W*D' PS='2*D+W' AD='W*D' PD='2*D+W'\n";
	}

	$s = $s . ".TEMP $temp\n";
	$s = $s . ".OP\n";
	$s = $s . ".OPTIONS LIST NODE POST CAPTAB\n";

	$s = $s . ".tran 'simt/10000' simt\n";
	$s = $s . ".print tran cap(gate)\n";
	$s = $s . ".print tran cap(drain)\n";
	$s = $s . ".print tran cap(source)\n";

	if ( $type eq "nmos" ) {
		$s = $s . ".measure tran leakage_current1 avg I(Vsource) FROM = t1s TO = t1e\n";
		$s = $s . ".measure tran leakage_current2 avg I(Vdrain) FROM = t2s TO = t2e\n";
		$s = $s . ".measure tran leakage_high Param=('(leakage_current1 + leakage_current2)/2')\n";
		$s = $s . ".measure tran leakage_current3 avg I(Vdrain) FROM = t6s TO = t6e\n";
		$s = $s . ".measure tran leakage_current4 avg I(Vsource) FROM = t7s TO = t7e\n";
		$s = $s . ".measure tran leakage_low Param=('(leakage_current3 + leakage_current4)/2')\n";
		$s = $s . ".measure tran c_gate_cmos avg cap(gate) FROM = t4s TO = t4e\n";
		$s = $s . ".measure tran c_drain_cmos avg cap(drain) FROM = t2s TO = t2e\n";
		$s = $s . ".measure tran c_gate_pass avg cap(gate) FROM = t4s TO = t5e\n";
		$s = $s . ".measure tran c_drain_pass1 avg cap(drain) FROM = t2s TO = t3e\n";
		$s = $s . ".measure tran c_drain_pass2 avg cap(drain) FROM = t5s TO = t5e\n";
		$s = $s . ".measure tran c_drain_pass Param=('(2*c_drain_pass1 + c_drain_pass2)/3')\n";
		$s = $s . ".measure tran c_source_pass1 avg cap(source) FROM = t1s TO = t1e\n";
		$s = $s . ".measure tran c_source_pass2 avg cap(source) FROM = t3s TO = t3e\n";
		$s = $s . ".measure tran c_source_pass3 avg cap(source) FROM = t5s TO = t5e\n";
		$s = $s . ".measure tran c_source_pass Param=('(c_source_pass1 + c_source_pass2 + c_source_pass3)/3')\n";
	}
	else {
		$s = $s . ".measure tran leakage_current1 avg I(Vsource) FROM = t3s TO = t3e\n";
		$s = $s . ".measure tran leakage_current2 avg I(Vdrain) FROM = t4s TO = t4e\n";
		$s = $s . ".measure tran leakage_high Param=('(leakage_current1 + leakage_current2)/2')\n";
		$s = $s . ".measure tran leakage_current3 avg I(Vdrain) FROM = t6s TO = t6e\n";
		$s = $s . ".measure tran leakage_current4 avg I(Vsource) FROM = t7s TO = t7e\n";
		$s = $s . ".measure tran leakage_low Param=('(leakage_current3 + leakage_current4)/2')\n";
		$s = $s . ".measure tran c_gate_cmos avg cap(gate) FROM = t3s TO = t3e\n";
		$s = $s . ".measure tran c_drain_cmos avg cap(drain) FROM = t1s TO = t1e\n";
		$s = $s . ".measure tran c_gate_pass avg cap(gate) FROM = t2s TO = t5e\n";
		$s = $s . ".measure tran c_drain_pass1 avg cap(drain) FROM = t1s TO = t1e\n";
		$s = $s . ".measure tran c_drain_pass2 avg cap(drain) FROM = t4s TO = t5e\n";
		$s = $s . ".measure tran c_drain_pass Param=('(c_drain_pass1 + 2*c_drain_pass2)/3')\n";
		$s = $s . ".measure tran c_source_pass1 avg cap(source) FROM = t1s TO = t1e\n";
		$s = $s . ".measure tran c_source_pass2 avg cap(source) FROM = t3s TO = t3e\n";
		$s = $s . ".measure tran c_source_pass3 avg cap(source) FROM = t5s TO = t5e\n";
		$s = $s . ".measure tran c_source_pass Param=('(c_source_pass1 + c_source_pass2 + c_source_pass3)/3')\n";
	}

	$s = $s . ".end\n\n";

	open( SP_FILE, "> $spice_file" );
	print SP_FILE $s;
	close(SP_FILE);

	system("$hspice $spice_file 1> $spice_out 2> /dev/null");

	my $spice_data;
	{
		local $/ = undef;
		open( SP_FILE, "$spice_out" );
		$spice_data = <SP_FILE>;
		close SP_FILE;
	}

	if ( $spice_data =~ /transient analysis.*?leakage_high\s*=\s*[+-]*(\S+)/s ) {
		$leakage_high = $1;
	}
	else {
		die "Could not find high leakage current in spice output.\n";
	}

	if ( $spice_data =~ /transient analysis.*?leakage_low\s*=\s*[+-]*(\S+)/s ) {
		$leakage_low = $1;
	}
	else {
		die "Could not find low leakage current in spice output.\n";
	}

	if ( $spice_data =~ /transient analysis.*?c_gate_cmos\s*=\s*(\S+)/s ) {
		$C_gate_cmos = $1;
	}
	else {
		die "Could not find gate capacitance in spice output.\n";
	}

	if ( $spice_data =~ /transient analysis.*?c_drain_cmos\s*=\s*(\S+)/s ) {
		$C_drain_cmos = $1;
	}
	else {
		die "Could not find drain capacitance in spice output.\n";
	}

	if ( $spice_data =~ /transient analysis.*?c_gate_pass\s*=\s*(\S+)/s ) {
		$C_gate_pass = $1;
	}
	else {
		die "Could not find gate capacitance in spice output.\n";
	}

	if ( $spice_data =~ /transient analysis.*?c_source_pass\s*=\s*(\S+)/s ) {
		$C_source_pass = $1;
	}
	else {
		die "Could not find source capacitance in spice output.\n";
	}

	if ( $spice_data =~ /transient analysis.*?c_drain_pass\s*=\s*(\S+)/s ) {
		$C_drain_pass = $1;
	}
	else {
		die "Could not find drain capacitance in spice output.\n";
	}

	return ( $leakage_high, $leakage_low, $C_gate_cmos, $C_drain_cmos, $C_gate_pass, $C_source_pass, $C_drain_pass );
}
