#!/usr/bin/perl

use File::Basename;
use File::Copy;
use Data::Dumper;
use Time::localtime;
use Switch;

## the defs

chomp($CURRENT_DIR	= `pwd`);
chomp($BIN_PATH_REL	= $CURRENT_DIR."/".`dirname $0`);
chomp($BIN_PATH		= `readlink -f $BIN_PATH_REL`);
chomp($OS			= `uname -o`);
chomp($CORES		= `ls -d /sys/devices/system/cpu/cpu[[:digit:]]* | wc -w`);

require "$BIN_PATH/ccd_defs.pl";

$BIN_PATH = $BIN_PATH."/bin";

my %objectStats	= {
	count			=> 0,
	good			=> 0,
	bad				=> 0,
	minWidth		=> 0,
	minHeight		=> 0,
	maxWidth 		=> 0,
	maxHeight		=> 0
};

my %jobOptions	= {
	resizeTo		=> 0,
	srcDir			=> $CURRENT_DIR
};

my %args = {};

my @resizeSizes = (orig, 2800, 2400, 2000, 1600, 1200, 1000, 800, 600);

$jobOptions{srcDir} = "$CURRENT_DIR";

sub parseArgs {

 ## defaults
	$args{"--start-with"}	= "resize";
	$args{"--end-with"}		= "poission";
	
	for($i = 0; $i <= $#ARGV; $i++) {
		if($ARGV[$i] =~ /^--[^a-z\-]*/){
			$args{"$ARGV[$i]"} = true;
			
			if(!($ARGV[$i+1] =~ /^--[^a-z\-]*/)) {
				if($ARGV[$i] eq "--resize-to"){
					if($ARGV[$i+1] eq "orig" || $ARGV[$i+1] =~ /^[0-9]*$/){
						$args{"--resize-to"} = $ARGV[$i+1];
					} else {
						die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1];
					}
				}
				
				if($ARGV[$i] eq "--start-with"){
					if($ARGV[$i+1] eq "resize" || $ARGV[$i+1] eq "getKeypoints" || $ARGV[$i+1] eq "match" || $ARGV[$i+1] eq "bundler" || $ARGV[$i+1] eq "cmvs" || $ARGV[$i+1] eq "pmvs" || $ARGV[$i+1] eq "poission"){
						$args{"--start-with"} = $ARGV[$i+1];
					} else {	
						die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1]."\n\t valid values are \"resize\", \"getKeypoints\", \"match\", \"bundler\", \"cmvs\", \"pmvs\", \"poission\"";	
					}
				}
				
				if($ARGV[$i] eq "--end-with"){
					if($ARGV[$i+1] eq "resize" || $ARGV[$i+1] eq "getKeypoints" || $ARGV[$i+1] eq "match" || $ARGV[$i+1] eq "bundler" || $ARGV[$i+1] eq "cmvs" || $ARGV[$i+1] eq "pmvs" || $ARGV[$i+1] eq "poission"){
						$args{"--end-with"} = $ARGV[$i+1];
					} else {	
						die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1]."\n\t valid values are \"resize\", \"getKeypoints\", \"match\", \"bundler\", \"cmvs\", \"pmvs\", \"poission\"";
					}
				}
								
				if($ARGV[$i] eq "--run-only"){
					if($ARGV[$i+1] eq "resize" || $ARGV[$i+1] eq "getKeypoints" || $ARGV[$i+1] eq "match" || $ARGV[$i+1] eq "bundler" || $ARGV[$i+1] eq "cmvs" || $ARGV[$i+1] eq "pmvs" || $ARGV[$i+1] eq "poission"){
						$args{"--start-with"} = $ARGV[$i+1];
						$args{"--end-with"} = $ARGV[$i+1];
					} else {	
						die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1]."\n\t valid values are \"resize\", \"getKeypoints\", \"match\", \"bundler\", \"cmvs\", \"pmvs\", \"poission\"";
					}
				}
			}
			
#			print "\n$ARGV[$i]: ".$args{"$ARGV[$i]"};
		}
	}
	
	if($args{"--help"}){		
		print "\nusgae run.pl [options] [path_to_images]";
		print "\n";
		print "\noptions:";
		print "\n        --help: ";
		print "\n                prints this screen";
		print "\n";
		print "\n   --resize-to: <integer>|\"orig\"";
		print "\n                will resize the images so that the maximum width/height of the images are smaller or equal to the specified number";
		print "\n                if \"--resize-to orig\" is used it will use the images without resizing";
		print "\n";
		print "\n  --start-with: \"resize\"|\"getKeypoints\"|\"match\"|\"bundler\"|\"cmvs\"|\"pmvs\"|\"poission\"";
		print "\n                will start the sript at the specified step";
		print "\n";
		print "\n    --end-with: \"resize\"|\"getKeypoints\"|\"match\"|\"bundler\"|\"cmvs\"|\"pmvs\"|\"poission\"";
		print "\n                will stop the sript after the specified step";
		print "\n";
		print "\n    --run-only: \"resize\"|\"getKeypoints\"|\"match\"|\"bundler\"|\"cmvs\"|\"pmvs\"|\"poission\"";
		print "\n                will only execute the specified step";
		print "\n                equal to --start-with <step> --end-with <step>";
		print "\n";
		exit;
	}
}

sub prepareObjects {
 ## get the source list	
	@source_files = `ls -1 | egrep "\.[jJ]{1}[pP]{1}[eE]{0,1}[gG]{1}"`;

	foreach $file (@source_files) {
		chomp($file);
		
		chomp($file_make		= `jhead $file | grep "Camera make"`);
		chomp($file_model		= `jhead $file | grep "Camera model"`);
		chomp($file_focal		= `jhead $file | grep "Focal length"`);
		chomp($file_ccd			= `jhead $file | grep "CCD width"`);
		chomp($file_resolution	= `jhead $file | grep "Resolution"`);
	
		my %fileObject = {};
	
		chomp(($fileObject{src})		= $file);
		chomp(($fileObject{base})		= $file);		
		$fileObject{base}				=~ s/\.[^\.]*$//;
	
		chomp(($fileObject{make})		= $file_make =~ /: ([^\n\r]*)/);
		chomp(($fileObject{model})		= $file_model =~ /: ([^\n\r]*)/);
	
		$fileObject{id}					= $fileObject{make}." ".$fileObject{model};
	
		($fileObject{width}, $fileObject{height})	= $file_resolution =~ /: ([0-9]*) x ([0-9]*)/;
	
		($fileObject{focal})			= $file_focal =~ /:[\ ]*([0-9\.]*)mm/;
		($fileObject{ccd})				= $file_ccd =~ /:[\ ]*([0-9\.]*)mm/;
	
		if(!$fileObject{ccd}){
			$fileObject{ccd} = $ccdWidths{$fileObject{id}};
		}
	
		if($fileObject{ccd} && $fileObject{focal} && $fileObject{width} && $fileObject{height}){
			if($fileObject{width} > $fileObject{height}){
				$fileObject{focalpx} = $fileObject{width} * ($fileObject{focal} / $fileObject{ccd});
			} else {
				$fileObject{focalpx} = $fileObject{height} * ($fileObject{focal} / $fileObject{ccd});
			}
			
			$fileObject{isOk} = true;
			$objectStats{good}++;
		} else {
			$fileObject{isOk} = false;
			$objectStats{bad}++;
			
			print "  no CCD width or focal length found for $fileObject{src} - ($fileObject{id})\n";
		}
	
		$objectStats{count}++;
	
		if($objectStats{minWidth} == 0) { $objectStats{minWidth} = $fileObject{width}; }
		if($objectStats{minHeight} == 0) { $objectStats{minHeight} = $fileObject{height}; }
	
		$objectStats{minWidth} = $objectStats{minWidth} < $fileObject{width} ? $objectStats{minWidth} : $fileObject{width};
		$objectStats{minHeight} = $objectStats{minHeight} < $fileObject{height} ? $objectStats{minHeight} : $fileObject{height};
		$objectStats{maxWidth} = $objectStats{maxWidth} > $fileObject{width} ? $objectStats{maxWidth} : $fileObject{width};
		$objectStats{maxHeight} = $objectStats{maxHeight} > $fileObject{height} ? $objectStats{maxHeight} : $fileObject{height};
			
		push(@objects, \%fileObject);
	}	
	
	print "\n  found $objectStats{good} usable images";
	
	if(!$args{"--resize-to"}){
		print "\n> please choose a max resolution to shrink the files to";
		print "\n  current images width ".($objectStats{minWidth} == $objectStats{maxWidth} ? "$objectStats{maxWidth}px" : " $objectStats{minWidth}px - $objectStats{maxWidth}px");
		print "\n  current images height ".($objectStats{minHeight} == $objectStats{maxHeight} ? "$objectStats{maxHeight}px" : "$objectStats{minHeight}px - $objectStats{maxHeight}px");
		print "\n";
		print "\n";
		print "\n    [0] original resolution << default";
		print "\n    [1] $resizeSizes[1] x $resizeSizes[1]px";
		print "\n    [2] $resizeSizes[2] x $resizeSizes[2]px";
		print "\n    [3] $resizeSizes[3] x $resizeSizes[3]px";
		print "\n    [4] $resizeSizes[4] x $resizeSizes[4]px";
		print "\n    [5] $resizeSizes[5] x $resizeSizes[5]px";
		print "\n    [6] $resizeSizes[6] x $resizeSizes[6]px";
		print "\n    [7]  $resizeSizes[7] x $resizeSizes[7]px";
		print "\n    [8]  $resizeSizes[8] x $resizeSizes[8]px";
		print "\n";
		print "\n   [0-9] > ";
	
		chomp($resizeInput = <>);
	
		if($resizeInput >= 0 && $resizeInput < 10){
			$jobOptions{resizeTo} = $resizeSizes[$resizeInput];
		} else {
			$jobOptions{resizeTo} = $resizeSizes[4];
		}
	} else {
		$jobOptions{resizeTo} = $args{"--resize-to"};
	}
	
	if($args{"--verbose"}){
		print "\n  using max image size of $jobOptions{resizeTo} x $jobOptions{resizeTo}";	
	}
	
	$jobOptions{jobDir} = "$jobOptions{srcDir}/reconstruction-with-image-size-$jobOptions{resizeTo}";
	
	$jobOptions{step_1_convert}			= "$jobOptions{jobDir}/_convert.templist.txt";
	$jobOptions{step_1_sift}			= "$jobOptions{jobDir}/_sift.templist.txt";
	$jobOptions{step_1_vlsift}			= "$jobOptions{jobDir}/_vlsift.templist.txt";
	$jobOptions{step_1_gzip}			= "$jobOptions{jobDir}/_gzip.templist.txt";
	
	$jobOptions{step_2_filelist}		= "$jobOptions{jobDir}/_filelist.templist.txt";
	$jobOptions{step_2_matches}			= "$jobOptions{jobDir}/matches.init.txt";
	
	$jobOptions{step_3_filelist}		= "$jobOptions{jobDir}/list.txt";
	$jobOptions{step_3_bundlerOptions}	= "$jobOptions{jobDir}/options.txt";
	
	mkdir($jobOptions{jobDir});
		
	foreach $fileObject (@objects) {
		if($fileObject->{isOk}){
			$fileObject->{step_0_resizedImage} = "$jobOptions{jobDir}/$fileObject->{base}.jpg";
			
			$fileObject->{step_1_pgmFile}	= "$jobOptions{jobDir}/$fileObject->{base}.pgm";
			$fileObject->{step_1_keyFile}	= "$jobOptions{jobDir}/$fileObject->{base}.key";
			$fileObject->{step_1_gzFile}	= "$jobOptions{jobDir}/$fileObject->{base}.key.gz";
		}
	}
	
#	exit
}

sub resize {	
	print "\n";
	print "\n  - preparing images - ";
	print "\n";

	chdir($jobOptions{jobDir});
	
	foreach $fileObject (@objects) {
		if($fileObject->{isOk}){			
			if($jobOptions{resizeTo} != "orig" && ($fileObject->{widht} > $jobOptions{resizeTo} || $fileObject->{height} > $jobOptions{resizeTo})){
				if($args{"--verbose"}){
					print "\n    resising $fileObject->{src} \tto $fileObject->{step_0_resizedImage}";
				}
				
				system("convert -resize $jobOptions{resizeTo}x$jobOptions{resizeTo} -quality 100 \"$jobOptions{srcDir}/$fileObject->{src}\" \"$fileObject->{step_0_resizedImage}\"");
			} else {
				if($args{"--verbose"}) {
					print "\n     copying $fileObject->{src} \tto $fileObject->{step_0_resizedImage}";
				}
				
				copy("$CURRENT_DIR/$fileObject->{src}", "$fileObject->{step_0_resizedImage}");
			}
		}
	}
	
	if($args{"--end-with"} ne "resize"){
		getKeypoints();
	}
}

sub getKeypoints {
	print "\n";
	print "\n  - finding keypoints - ";
	print "\n";

	chdir($jobOptions{jobDir});
	
	$pgmJobs	= "";
#	$siftJobs	= "";
	$vlsiftJobs	= "";
	$gzJobs		= "";
	
	foreach $fileObject (@objects) {
		if($fileObject->{isOk}){			
			$pgmJobs	.= "convert -format pgm \"$fileObject->{step_0_resizedImage}\" \"$fileObject->{step_1_pgmFile}\"\n";
#			$siftJobs	.= "$BIN_PATH/sift < \"$fileObject->{step_1_pgmFile}\" > \"$fileObject->{step_1_keyFile}.lowe\"\n";
			$vlsiftJobs	.= "$BIN_PATH/vlsift \"$fileObject->{step_1_pgmFile}\" -o \"$fileObject->{step_1_keyFile}.sift\" > /dev/null && perl $BIN_PATH/../convert_vlsift_to_lowesift.pl \"$fileObject->{base}\"\n";
			$gzJobs		.= "gzip -f \"$fileObject->{step_1_keyFile}\"\n";
		}
	}
	
	system("echo \"$pgmJobs\"		> $jobOptions{step_1_convert}");
#	system("echo \"$siftJobs\"		> $jobOptions{step_1_sift}");
	system("echo \"$vlsiftJobs\"	> $jobOptions{step_1_vlsift}");
	system("echo \"$gzJobs\" 		> $jobOptions{step_1_gzip}");
	
	system("\"$BIN_PATH/parallel\" -j+0 < \"$jobOptions{step_1_convert}\"");
#	system("\"$BIN_PATH/parallel\" -j+0 < \"$jobOptions{step_1_sift}\"");
	system("\"$BIN_PATH/parallel\" -j+0 < \"$jobOptions{step_1_vlsift}\"");
	system("\"$BIN_PATH/parallel\" -j+0 < \"$jobOptions{step_1_gzip}\"");
	
#	system("rm -f \"$jobOptions{jobDir}/\"*.pgm");
#	system("rm -f \"$jobOptions{jobDir}/\"*.key");
#	system("rm -f \"$jobOptions{jobDir}/\"*.key.sift");
	
	if($args{"--end-with"} ne "getKeypoints"){
		match();
	}
}

sub match {
	print "\n";
	print "\n  - matching keypoints - ";
	print "\n";

	chdir($jobOptions{jobDir});
	
	$filesList = "";
	
	foreach $fileObject (@objects) {
		if($fileObject->{isOk}){
			if($fileObject->{isOk}){
				$filesList		.= "\"$fileObject->{step_1_keyFile}\"\n";
			}
		}
	}
	
	system("echo \"$filesList\" 	> $jobOptions{step_2_filelist}	 ");
	system("\"$BIN_PATH/KeyMatchFull\" \"$jobOptions{step_2_filelist}\" \"$jobOptions{step_2_matches}\"	");
	
	if($args{"--end-with"} ne "match"){
		bundler();
	}
}

sub bundler {
	print "\n";
	print "\n  - running bundler - ";
	print "\n";

	chdir($jobOptions{jobDir});
	
	mkdir($jobOptions{jobDir}."/bundle");
	mkdir($jobOptions{jobDir}."/pmvs");
	mkdir($jobOptions{jobDir}."/pmvs/txt");
	mkdir($jobOptions{jobDir}."/pmvs/visualize");
	mkdir($jobOptions{jobDir}."/pmvs/models");
	
	$filesList = "";
	
	foreach $fileObject (@objects) {
		if($fileObject->{isOk}){
			if($fileObject->{isOk}){
				$filesList		.= sprintf("\./%s.jpg 0 %0.5f\n", $fileObject->{base}, $fileObject->{focalpx});
			}
		}
	}
	
	chomp($filesList);
	
	$bundlerOptions  = "--match_table matches.init.txt\n";
	$bundlerOptions .= "--output bundle.out\n";
	$bundlerOptions .= "--output_all bundle_\n";
	$bundlerOptions .= "--output_dir bundle\n";
	$bundlerOptions .= "--variable_focal_length\n";
	$bundlerOptions .= "--use_focal_estimate\n";
	$bundlerOptions .= "--constrain_focal\n";
	$bundlerOptions .= "--constrain_focal_weight 0.0001\n";
	$bundlerOptions .= "--estimate_distortion\n";
	$bundlerOptions .= "--run_bundle";
		
	system("echo \"$bundlerOptions\" > \"$jobOptions{step_3_bundlerOptions}\"");
	system("echo \"$filesList\" > \"$jobOptions{step_3_filelist}\"");
	
	system("\"$BIN_PATH/bundler\" \"$jobOptions{step_3_filelist}\" --options_file \"$jobOptions{step_3_bundlerOptions}\" > bundle/out");

	system("\"$BIN_PATH/Bundle2PMVS\" \"$jobOptions{step_3_filelist}\" bundle/bundle.out");
	
	system("\"$BIN_PATH/RadialUndistort\" \"$jobOptions{step_3_filelist}\" bundle/bundle.out pmvs");

	$i = 0;
	
	foreach $fileObject (@objects) {
		if($fileObject->{isOk}){
			if($fileObject->{isOk}){
				if (-e "pmvs/$fileObject->{base}.rd.jpg"){
					$nr = sprintf("%08d", $i++);
				
					system("mv pmvs/$fileObject->{base}.rd.jpg pmvs/visualize/$nr.jpg");
					system("mv pmvs/$nr.txt pmvs/txt/$nr.txt");
				}
			}
		}
	}
	
	system("\"$BIN_PATH/Bundle2Vis\" pmvs/bundle.rd.out pmvs/vis.dat");
	
	if($args{"--end-with"} ne "bundler"){
		cmvs();
	}
}

sub cmvs {
	print "\n";
	print "\n  - running cmvs - ";
	print "\n";

	chdir($jobOptions{jobDir});
	
	system("\"$BIN_PATH/cmvs\" pmvs/ 100 4");
	system("\"$BIN_PATH/genOption\" pmvs/ ");
	
	if($args{"--end-with"} ne "cmvs"){
		pmvs();
	}
}

sub pmvs {
	print "\n";
	print "\n  - running pmvs - ";
	print "\n";

	chdir($jobOptions{jobDir});
	
	system("\"$BIN_PATH/pmvs2\" pmvs/ option-0000");
	
	if($args{"--end-with"} ne "pmvs"){
		poission();
	}
}

sub poission {
	print "\n";
	print "\n  - running poission reconstruction - ";
	print "\n";
	
	foreach $fileObject (@objects) {
		if($fileObject->{isOk}){
			
		}
	}
}

parseArgs();
prepareObjects();
	
chdir($jobOptions{jobDir});
	
switch ($args{"--start-with"}) {
	case "resize"		{ resize();			}
	case "getKeypoints"	{ getKeypoints();	}
	case "match"		{ match();			}
	case "bundler"		{ bundler();		}
	case "cmvs"			{ cmvs();			}
	case "pmvs"			{ pmvs();			}
}

print "\n";
print "\n  - done - ";
print "\n";
