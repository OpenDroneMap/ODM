#!/usr/bin/perl

##
##  created by Daniel Schwarz/daniel.schwarz@topoi.org
##  released under Creative Commons/CC-BY
##  Attribution
##

use File::Basename;
use File::Copy;
use File::Spec;
use Data::Dumper;
use Time::localtime;
use Switch;
use POSIX qw(strftime);

## the defs

chomp($CURRENT_DIR  = `pwd`);
chomp($BIN_PATH_REL = `dirname $0`);
chomp($OS           = `uname -o`);
chomp($CORES        = `ls -d /sys/devices/system/cpu/cpu[[:digit:]]* | wc -w`);

if(!File::Spec->file_name_is_absolute($BIN_PATH_REL)){
	$BIN_PATH_ABS = File::Spec->rel2abs($BIN_PATH_REL);
} else {
	$BIN_PATH_ABS = File::Spec->rel2abs($BIN_PATH_REL);
}

require "$BIN_PATH_ABS/ccd_defs.pl";

$BIN_PATH = $BIN_PATH_ABS."/bin";

my %objectStats    = {
    count           => 0,
    good            => 0,
    bad             => 0,
    minWidth        => 0,
    minHeight       => 0,
    maxWidth        => 0,
    maxHeight       => 0

};

my %jobOptions    = {
    resizeTo        => 0,
    srcDir          => $CURRENT_DIR
};

my %args = {};

$jobOptions{srcDir} = "$CURRENT_DIR";

sub run {
    system($_[0]);
    
    if($? != 0){
        die "\n\nquitting cause: \n\t$_[0]\nreturned with code ".$?."\n";
    }
}
sub now {
	system("echo `date`");
}

sub parseArgs {

 ## defaults
    $args{"--match-size"}            = "200";

    $args{"--resize-to"}             = "3000";
    
    $args{"--start-with"}            = "resize";
    $args{"--end-with"}              = "pmvs";
    
    $args{"--cmvs-maxImages"}        = 100;
	
    $args{"--matcher-ratio"}     	 = 0.6;
    $args{"--matcher-threshold"}     = 2.0;
    
    $args{"--pmvs-level"}            = 1;
    $args{"--pmvs-csize"}            = 2;
    $args{"--pmvs-threshold"}        = 0.7;
    $args{"--pmvs-wsize"}            = 7;
    $args{"--pmvs-minImageNum"}      = 3;
    
    for($i = 0; $i <= $#ARGV; $i++) {
        if($ARGV[$i] =~ /^--[^a-z\-]*/){
            $args{"$ARGV[$i]"} = true;
            
            if(!($ARGV[$i+1] =~ /^--[^a-z\-]*/)) {
                if($ARGV[$i] eq "--resize-to"){
                    if($ARGV[$i+1] eq "orig" || $ARGV[$i+1] =~ /^[0-9]*$/){
                        $args{$ARGV[$i]} = $ARGV[$i+1];
                    } else {
                        die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1];
                    }
                }
                if($ARGV[$i] eq "--start-with"){
                    if($ARGV[$i+1] eq "resize" || $ARGV[$i+1] eq "getKeypoints" || $ARGV[$i+1] eq "match" || $ARGV[$i+1] eq "bundler" || $ARGV[$i+1] eq "cmvs" || $ARGV[$i+1] eq "pmvs"){
                        $args{$ARGV[$i]} = $ARGV[$i+1];
                    } else {    
                        die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1]."\n\t valid values are \"resize\", \"getKeypoints\", \"match\", \"bundler\", \"cmvs\", \"pmvs\"";    
                    }
                }
                if($ARGV[$i] eq "--end-with"){
                    if($ARGV[$i+1] eq "resize" || $ARGV[$i+1] eq "getKeypoints" || $ARGV[$i+1] eq "match" || $ARGV[$i+1] eq "bundler" || $ARGV[$i+1] eq "cmvs" || $ARGV[$i+1] eq "pmvs"){
                        $args{$ARGV[$i]} = $ARGV[$i+1];
                    } else {    
                        die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1]."\n\t valid values are \"resize\", \"getKeypoints\", \"match\", \"bundler\", \"cmvs\", \"pmvs\"";
                    }
                }
                if($ARGV[$i] eq "--run-only"){
                    if($ARGV[$i+1] eq "resize" || $ARGV[$i+1] eq "getKeypoints" || $ARGV[$i+1] eq "match" || $ARGV[$i+1] eq "bundler" || $ARGV[$i+1] eq "cmvs" || $ARGV[$i+1] eq "pmvs"){
                        $args{"--start-with"} = $ARGV[$i+1];
                        $args{"--end-with"} = $ARGV[$i+1];
                    } else {    
                        die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1]."\n\t valid values are \"resize\", \"getKeypoints\", \"match\", \"bundler\", \"cmvs\", \"pmvs\"";
                    }
                }
	            if($ARGV[$i] eq "--matcher-threshold"){
                    if($ARGV[$i+1] =~ /^-?[0-9]*\.?[0-9]*$/){
                        $args{$ARGV[$i]} = $ARGV[$i+1];
                    } else {
                        die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1];
                    }
                }
	            if($ARGV[$i] eq "--matcher-ratio"){
                    if($ARGV[$i+1] =~ /^-?[0-9]*\.?[0-9]*$/){
                        $args{$ARGV[$i]} = $ARGV[$i+1];
                    } else {
                        die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1];
                    }
                }
                if($ARGV[$i] eq "--cmvs-maxImages"){
                    if($ARGV[$i+1] =~ /^[0-9]*$/){
                        $args{$ARGV[$i]} = $ARGV[$i+1];
                    } else {
                        die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1];
                    }
                }
                if($ARGV[$i] eq "--pmvs-level"){
                    if($ARGV[$i+1] =~ /^[0-9]*$/){
                        $args{$ARGV[$i]} = $ARGV[$i+1];
                    } else {
                        die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1];
                    }
                }
                if($ARGV[$i] eq "--pmvs-csize"){
                    if($ARGV[$i+1] =~ /^[0-9]*$/){
                        $args{$ARGV[$i]} = $ARGV[$i+1];
                    } else {
                        die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1];
                    }
                }
                if($ARGV[$i] eq "--pmvs-threshold"){
                    if($ARGV[$i+1] =~ /^-?[0-9]*\.?[0-9]*$/){
                        $args{$ARGV[$i]} = $ARGV[$i+1];
                    } else {
                        die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1];
                    }
                }
                if($ARGV[$i] eq "--pmvs-wsize"){
                    if($ARGV[$i+1] =~ /^[0-9]*$/){
                        $args{$ARGV[$i]} = $ARGV[$i+1];
                    } else {
                        die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1];
                    }
                }
                if($ARGV[$i] eq "--pmvs-minImageNum"){
                    if($ARGV[$i+1] =~ /^[0-9]*$/){
                        $args{$ARGV[$i]} = $ARGV[$i+1];
                    } else {
                        die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1];
                    }
                }
                
                if($ARGV[$i] eq "--force-focal"){
                    if($ARGV[$i+1] =~ /^[0-9]*\.?[0-9]*$/){
                        $args{$ARGV[$i]} = $ARGV[$i+1];
                    } else {
                        die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1];
                    }
                }
                if($ARGV[$i] eq "--force-ccd"){
                    if($ARGV[$i+1] =~ /^[0-9]*\.?[0-9]*$/){
                        $args{$ARGV[$i]} = $ARGV[$i+1];
                    } else {
                        die "\n invalid parameter for \"".$ARGV[$i]."\": ".$ARGV[$i+1];
                    }
                }
            }
        }
    }
    
    if($args{"--help"}){        
        print "\nusage: run.pl [options]";
        print "\nit should be run from the folder containing the images to be reconstructed";
        print "\n";
        print "\noptions:";
        print "\n              --help: ";
        print "\n                      prints this screen";
        print "\n  ";
                   
        print "\n         --resize-to: <positive integer|\"orig\">";
        print "\n             default: 3000";
        print "\n                      will resize the images so that the maximum width/height of the images are smaller or equal to the specified number";
        print "\n                      if \"--resize-to orig\" is used it will use the images without resizing";
        print "\n  ";
                   
        print "\n        --start-with: <\"resize\"|\"getKeypoints\"|\"match\"|\"bundler\"|\"cmvs\"|\"pmvs\">";
        print "\n             default: resize";
        print "\n                      will start the sript at the specified step";
        print "\n  ";
                   
        print "\n          --end-with: <\"resize\"|\"getKeypoints\"|\"match\"|\"bundler\"|\"cmvs\"|\"pmvs\">";
        print "\n             default: pmvs";
        print "\n                      will stop the sript after the specified step";
        print "\n  ";
                   
        print "\n          --run-only: <\"resize\"|\"getKeypoints\"|\"match\"|\"bundler\"|\"cmvs\"|\"pmvs\">";
        print "\n                      will only execute the specified step";
        print "\n                      equal to --start-with <step> --end-with <step>";
        print "\n  ";
                   
        print "\n       --force-focal: <positive float>";
        print "\n                      override the focal length information for the images";
        print "\n  ";
                   
        print "\n         --force-ccd: <positive float>";
        print "\n                      override the ccd width information for the images";
        print "\n  ";
                   
        print "\n --matcher-threshold: <float> (percent)";
        print "\n             default: 2.0";
        print "\n                      ignore matched keypoints if the two images share less than <float> percent of keypoints";
        print "\n  ";
                   
        print "\n     --matcher-ratio: <float";
        print "\n             default: 0.6";
        print "\n                      ratio of the distance to the next best matched keypoint";
        print "\n  ";
		
		
                   
        print "\n    --cmvs-maxImages: <positive integer>";
        print "\n             default: 100";
        print "\n                      the maximum number of images per cluster";
        print "\n  ";
                   
        print "\n        --pmvs-level: <positive integer>";
        print "\n             default: 1";
                   
        print "\n        --pmvs-csize: <positive integer>";
        print "\n             default: 2";
                   
        print "\n    --pmvs-threshold: <float: -1.0 <= x <= 1.0>";
        print "\n             default: 0.7";
                   
        print "\n        --pmvs-wsize: <positive integer>";
        print "\n             default: 7";
                   
        print "\n  --pmvs-minImageNum: <positive integer>";
        print "\n             default: 3";
        print "\n                      see http://grail.cs.washington.edu/software/pmvs/documentation.html for an explanation of these parameters";
        print "\n";
        
        exit;
    }
    
    print "\n  - configuration:";
    
    foreach $args_key (sort keys %args) {
        if($args{$args_key} ne ""){
            print "\n    $args_key: $args{$args_key}";
        }
    }
    
    print "\n";
    print "\n";
}

sub prepareObjects {
 ## get the source list    
    @source_files = `ls -1 | egrep "\.[jJ]{1}[pP]{1}[eE]{0,1}[gG]{1}"`;
    
    print "\n  - source files - "; now(); print "\n";

    foreach $file (@source_files) {
        chomp($file);
        
        chomp($file_make        = `jhead \"$file\" | grep "Camera make"`);
        chomp($file_model       = `jhead \"$file\" | grep "Camera model"`);
        chomp($file_focal       = `jhead \"$file\" | grep "Focal length"`);
        chomp($file_ccd         = `jhead \"$file\" | grep "CCD width"`);
        chomp($file_resolution  = `jhead \"$file\" | grep "Resolution"`);
    
        my %fileObject = {};
    
        chomp(($fileObject{src})        = $file);
        chomp(($fileObject{base})       = $file);
        $fileObject{base}               =~ s/\.[^\.]*$//;
    
        chomp(($fileObject{make})       = $file_make =~ /: ([^\n\r]*)/);
        chomp(($fileObject{model})      = $file_model =~ /: ([^\n\r]*)/);
        
        $fileObject{make} =~ s/^\s+//;		$fileObject{make} =~  s/\s+$//;
        $fileObject{model} =~ s/^\s+//;		$fileObject{model} =~  s/\s+$//;
    
        $fileObject{id}                 = $fileObject{make}." ".$fileObject{model};
    
        ($fileObject{width}, $fileObject{height})    = $file_resolution =~ /: ([0-9]*) x ([0-9]*)/;
            
        if(!$args{"--force-focal"}){
            ($fileObject{focal})      = $file_focal =~ /:[\ ]*([0-9\.]*)mm/;
        } else {
            $fileObject{focal}        = $args{"--force-focal"};
        }
        
        if(!$args{"--force-ccd"}){
            ($fileObject{ccd})        = $file_ccd =~ /:[\ ]*([0-9\.]*)mm/;
            
            if(!$fileObject{ccd}){;
                $fileObject{ccd}      = $ccdWidths{$fileObject{id}};
            }
        } else {
            $fileObject{ccd}          = $args{"--force-ccd"};
        }
    
        if($fileObject{ccd} && $fileObject{focal} && $fileObject{width} && $fileObject{height}){
            if($fileObject{width} > $fileObject{height}){
                $fileObject{focalpx} = $fileObject{width} * ($fileObject{focal} / $fileObject{ccd});
            } else {
                $fileObject{focalpx} = $fileObject{height} * ($fileObject{focal} / $fileObject{ccd});
            }
            
            $fileObject{isOk} = true;
            $objectStats{good}++;
            
            print "\n     using $fileObject{src}     dimensions: $fileObject{width}x$fileObject{height} / focal: $fileObject{focal}mm / ccd: $fileObject{ccd}mm";
        } else {
            $fileObject{isOk} = false;
            $objectStats{bad}++;
            
            print "\n    no CCD width or focal length found for $fileObject{src} - camera: \"$fileObject{id}\"";
        }
    
        $objectStats{count}++;
    
        if($objectStats{minWidth} == 0)  { $objectStats{minWidth}  = $fileObject{width}; }
        if($objectStats{minHeight} == 0) { $objectStats{minHeight} = $fileObject{height}; }
    
        $objectStats{minWidth}  = $objectStats{minWidth}  < $fileObject{width}  ? $objectStats{minWidth}  : $fileObject{width};
        $objectStats{minHeight} = $objectStats{minHeight} < $fileObject{height} ? $objectStats{minHeight} : $fileObject{height};
        $objectStats{maxWidth}  = $objectStats{maxWidth}  > $fileObject{width}  ? $objectStats{maxWidth}  : $fileObject{width};
        $objectStats{maxHeight} = $objectStats{maxHeight} > $fileObject{height} ? $objectStats{maxHeight} : $fileObject{height};
            
        push(@objects, \%fileObject);
    }
    
    
    
    if(!$objectStats{good}){
        print "\n\n    found no usable images - quitting\n";
        die;
    } else {
        print "\n\n    found $objectStats{good} usable images";
    }
    
    print "\n";
    
    $jobOptions{resizeTo} = $args{"--resize-to"};

    print "\n  using max image size of $jobOptions{resizeTo} x $jobOptions{resizeTo}";    
    
    $jobOptions{jobDir} = "$jobOptions{srcDir}/reconstruction-with-image-size-$jobOptions{resizeTo}";
    
    $jobOptions{step_1_convert}         = "$jobOptions{jobDir}/_convert.templist.txt";
    $jobOptions{step_1_vlsift}          = "$jobOptions{jobDir}/_vlsift.templist.txt";
    $jobOptions{step_1_gzip}            = "$jobOptions{jobDir}/_gzip.templist.txt";
    
    $jobOptions{step_2_filelist}        = "$jobOptions{jobDir}/_filelist.templist.txt";
    $jobOptions{step_2_macthes_jobs}    = "$jobOptions{jobDir}/_matches_jobs.templist.txt";
    $jobOptions{step_2_matches_dir}  	= "$jobOptions{jobDir}/matches";
    $jobOptions{step_2_matches}         = "$jobOptions{jobDir}/matches.init.txt";
    
    $jobOptions{step_3_filelist}        = "$jobOptions{jobDir}/list.txt";
    $jobOptions{step_3_bundlerOptions}  = "$jobOptions{jobDir}/options.txt";
    
    mkdir($jobOptions{jobDir});
        
    foreach $fileObject (@objects) {
        if($fileObject->{isOk}){
            $fileObject->{step_0_resizedImage}  = "$jobOptions{jobDir}/$fileObject->{base}.jpg";
            
            $fileObject->{step_1_pgmFile}       = "$jobOptions{jobDir}/$fileObject->{base}.pgm";
            $fileObject->{step_1_keyFile}       = "$jobOptions{jobDir}/$fileObject->{base}.key";
            $fileObject->{step_1_gzFile}        = "$jobOptions{jobDir}/$fileObject->{base}.key.gz";
        }
    }
    
#    exit
}

sub resize {
    print "\n";
    print "\n  - preparing images - "; now(); print "\n";
    print "\n";

    chdir($jobOptions{jobDir});
    
    foreach $fileObject (@objects) {
        if($fileObject->{isOk}){
			unless (-e "$fileObject->{step_0_resizedImage}"){
          	  if($jobOptions{resizeTo} != "orig" && (($fileObject->{width} > $jobOptions{resizeTo}) || ($fileObject->{height} > $jobOptions{resizeTo}))){
	                print "\n    resizing $fileObject->{src} \tto $fileObject->{step_0_resizedImage}";
                
	                run("convert -resize $jobOptions{resizeTo}x$jobOptions{resizeTo} -quality 100 \"$jobOptions{srcDir}/$fileObject->{src}\" \"$fileObject->{step_0_resizedImage}\"");

	            } else {
	                print "\n     copying $fileObject->{src} \tto $fileObject->{step_0_resizedImage}";
                
	                copy("$CURRENT_DIR/$fileObject->{src}", "$fileObject->{step_0_resizedImage}");
	            }
			} else {	
	          	print "\n     using existing $fileObject->{src} \tto $fileObject->{step_0_resizedImage}";
			}
            
            chomp($file_resolution    = `jhead \"$fileObject->{step_0_resizedImage}\" | grep "Resolution"`);
            ($fileObject->{width}, $fileObject->{height})    = $file_resolution =~ /: ([0-9]*) x ([0-9]*)/;
            print "\t ($fileObject->{width} x $fileObject->{height})";
        }
    }
    
    if($args{"--end-with"} ne "resize"){
        getKeypoints();
    }
}

sub getKeypoints {
    print "\n";
    print "\n  - finding keypoints - "; now(); print "\n";
    print "\n\n";
    
    chdir($jobOptions{jobDir});
    
    $vlsiftJobs    = "";
    
	$c = 0;

    foreach $fileObject (@objects) {
		$c = $c+1;
	
        if($fileObject->{isOk}){
            if($args{"--lowe-sift"}){
                $vlsiftJobs    .= "echo -n \" $c/$objectStats{good} - \" && convert -format pgm \"$fileObject->{step_0_resizedImage}\" \"$fileObject->{step_1_pgmFile}\"";
                $vlsiftJobs    .= " && \"$BIN_PATH/sift\" < \"$fileObject->{step_1_pgmFile}\" > \"$fileObject->{step_1_keyFile}\"";
                $vlsiftJobs    .= " && gzip -f \"$fileObject->{step_1_keyFile}\"";
                $vlsiftJobs    .= " && rm -f \"$fileObject->{step_1_pgmFile}\"";
                $vlsiftJobs    .= " && rm -f \"$fileObject->{step_1_keyFile}.sift\"\n";
            } else {
				unless (-e "$jobOptions{jobDir}/$fileObject->{base}.key.bin") {
	                $vlsiftJobs    .= "echo -n \" $c/$objectStats{good} - \" && convert -format pgm \"$fileObject->{step_0_resizedImage}\" \"$fileObject->{step_1_pgmFile}\"";
	                $vlsiftJobs    .= " && \"$BIN_PATH/vlsift\" \"$fileObject->{step_1_pgmFile}\" -o \"$fileObject->{step_1_keyFile}.sift\" > /dev/null && perl \"$BIN_PATH/../convert_vlsift_to_lowesift.pl\" \"$jobOptions{jobDir}/$fileObject->{base}\"";
	                $vlsiftJobs    .= " && gzip -f \"$fileObject->{step_1_keyFile}\"";
	                $vlsiftJobs    .= " && rm -f \"$fileObject->{step_1_pgmFile}\"";
	                $vlsiftJobs    .= " && rm -f \"$fileObject->{step_1_keyFile}.sift\"\n";
				} else {
					print "using existing $jobOptions{jobDir}/$fileObject->{base}.key.bin\n";
				}
            }
        }
    }
    
    open (SIFT_DEST, ">$jobOptions{step_1_vlsift}");
    print SIFT_DEST $vlsiftJobs;
    close(SIFT_DEST);
    
    run("\"$BIN_PATH/parallel\" --halt-on-error 1 -j3 < \"$jobOptions{step_1_vlsift}\"");
    
    if($args{"--end-with"} ne "getKeypoints"){
        match();
    }
}

sub match {
    print "\n";
    print "\n  - matching keypoints - "; now(); print "\n";
    print "\n";

    chdir($jobOptions{jobDir});
    mkdir($jobOptions{step_2_matches_dir});
    
    $matchesJobs = "";

	my $c = 0;
	my $t = ($objectStats{good}-1) * ($objectStats{good}/2);
	
	for (my $i = 0; $i < $objectStats{good}; $i++) {
		for (my $j = $i+1; $j < $objectStats{good}; $j++) {
			$c++;
			unless (-e "$jobOptions{step_2_matches_dir}/$i-$j.txt"){ 
				
			   
				$matchesJobs        .=  "echo -n \".\" && touch \"$jobOptions{step_2_matches_dir}/$i-$j.txt\" && \"$BIN_PATH/KeyMatch\" \"@objects[$i]->{step_1_keyFile}\" \"@objects[$j]->{step_1_keyFile}\" \"$jobOptions{step_2_matches_dir}/$i-$j.txt\" $args{'--matcher-ratio'} $args{'--matcher-threshold'}\n";
			}
		}
	}
	
    open (MATCH_DEST, ">$jobOptions{step_2_macthes_jobs}");
    print MATCH_DEST $matchesJobs;
    close(MATCH_DEST);
	
    run("\"$BIN_PATH/parallel\" --halt-on-error 1 -j+0 < \"$jobOptions{step_2_macthes_jobs}\"");
	
	run("rm -f \"$jobOptions{step_2_matches}\"");
	
	for (my $i = 0; $i < $objectStats{good}; $i++) {
		for (my $j = $i+1; $j < $objectStats{good}; $j++) {
			$c++;
			
			if (-e "$jobOptions{step_2_matches_dir}/$i-$j.txt" && (-s "$jobOptions{step_2_matches_dir}/$i-$j.txt") > 0) {
				run("echo \"$i $j\" >> \"$jobOptions{step_2_matches}\" && cat \"$jobOptions{step_2_matches_dir}/$i-$j.txt\" >> \"$jobOptions{step_2_matches}\"");
			}
		}
	}
	
    foreach $fileObject (@objects) {
        if($fileObject->{isOk}){
            if($fileObject->{isOk}){
                $filesList        .= "$fileObject->{step_1_keyFile}\n";
            }
        }
    }
	
    open (MATCH_DEST, ">$jobOptions{step_2_filelist}");
    print MATCH_DEST $filesList;
    close(MATCH_DEST);
    
 #   run("\"$BIN_PATH/KeyMatchFull\" \"$jobOptions{step_2_filelist}\" \"$jobOptions{step_2_matches}\"    ");
    
    if($args{"--end-with"} ne "match"){
        bundler();
    }
}

sub bundler {
    print "\n";
    print "\n  - running bundler - "; now(); print "\n";
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
                $filesList        .= sprintf("\./%s.jpg 0 %0.5f\n", $fileObject->{base}, $fileObject->{focalpx});
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
    $bundlerOptions .= "--constrain_focal_weight 0.01\n";
    $bundlerOptions .= "--estimate_distortion\n";
    $bundlerOptions .= "--run_bundle";
        
    system("echo \"$bundlerOptions\" > \"$jobOptions{step_3_bundlerOptions}\"");

    open (BUNDLER_DEST, ">$jobOptions{step_3_filelist}");
    print BUNDLER_DEST $filesList;
    close(BUNDLER_DEST);
    
    run("\"$BIN_PATH/bundler\" \"$jobOptions{step_3_filelist}\" --options_file \"$jobOptions{step_3_bundlerOptions}\" > bundle/out");
    run("\"$BIN_PATH/Bundle2PMVS\" \"$jobOptions{step_3_filelist}\" bundle/bundle.out");
    run("\"$BIN_PATH/RadialUndistort\" \"$jobOptions{step_3_filelist}\" bundle/bundle.out pmvs");
    
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
    print "\n  - running cmvs - "; now(); print "\n";
    print "\n";

    chdir($jobOptions{jobDir});
    
    run("\"$BIN_PATH/cmvs\" pmvs/ $args{'--cmvs-maxImages'} $CORES");
    run("\"$BIN_PATH/genOption\" pmvs/ $args{'--pmvs-level'} $args{'--pmvs-csize'} $args{'--pmvs-threshold'} $args{'--pmvs-wsize'} $args{'--pmvs-minImageNum'} $CORES");
    
    if($args{"--end-with"} ne "cmvs"){
        pmvs();
    }
}

sub pmvs {
    print "\n";
    print "\n  - running pmvs - ";
    print "\n";

    chdir($jobOptions{jobDir});
    
    run("\"$BIN_PATH/pmvs2\" pmvs/ option-0000");
    
    system("cp -Rf \"$jobOptions{jobDir}/pmvs/models\" \"$jobOptions{jobDir}-results\"");
}

parseArgs();
prepareObjects();
    
chdir($jobOptions{jobDir});
    
switch ($args{"--start-with"}) {
    case "resize"          { resize();          }
    case "getKeypoints"    { getKeypoints();    }
    case "match"           { match();           }
    case "bundler"         { bundler();         }
    case "cmvs"            { cmvs();            }
    case "pmvs"            { pmvs();            }
}

print "\n";
print "\n  - done - "; now(); print "\n";
print "\n";
