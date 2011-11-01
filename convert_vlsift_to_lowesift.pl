#!/usr/local/bin/perl

$filename_base	= $ARGV[0];
                           
$write_binary	= 1;    
         
$filename_src		= $filename_base.".key.sift";
$filename_dest_bin	= $filename_base.".key.bin";
$filename_dest_key	= $filename_base.".key";
$filename_image		= $filename_base.".pgm";   

open (DEST_BIN, ">$filename_dest_bin");
open (DEST_KEY, ">$filename_dest_key");

open (SRC, "$filename_src");

$linecount = 0;
$linecount += tr/\n/\n/ while sysread(SRC, $_, 2 ** 16);

printf ("%d", $linecount);

if($write_binary){              
	seek(SRC, 0, 0);
	
	print DEST_BIN pack("L", $linecount);

	while ($record = <SRC>) {  
		@parts = split(/ /, $record);             
		
		if(@parts[3] > 3.141){
			@parts[3] -= 6.282;
		} 
		        
		@parts[3] *= -1;  
		
       	@tmp	  = @parts[0];
		@parts[0] = @parts[1];
	    @parts[1] = @tmp;               
			
	   	for ($count = 4; $count < 132; $count += 8) {
			@tmp			 = @parts[$count+7];
			@parts[$count+7] = @parts[$count+1];
			@parts[$count+1] = @tmp;

			@tmp			 = @parts[$count+6];
			@parts[$count+6] = @parts[$count+2];
			@parts[$count+2] = @tmp;

			@tmp			 = @parts[$count+3];
			@parts[$count+3] = @parts[$count+5];
			@parts[$count+5] = @tmp;
		}            
		
		print DEST_BIN pack("f4 C128", @parts);
	}
} 

	seek(SRC, 0, 0);

	print DEST_KEY $linecount, " 128\n";  

	while ($record = <SRC>) {
		@parts = split(/ /, $record);
	
		$counter = 0;
	
		if(@parts[3] > 3.141){
			@parts[3] -= 6.282;
		}
	         	
		@parts[3] *= -1;              
	
		printf (DEST_KEY "%.3f %.3f %.3f %.3f", @parts[1], @parts[0], @parts[2], @parts[3]);
	
		shift(@parts);
		shift(@parts);
		shift(@parts);
		shift(@parts);
	
		for ($count = 0; $count < 128; $count += 8) {
			@tmp			 = @parts[$count+7];
			@parts[$count+7] = @parts[$count+1];
			@parts[$count+1] = @tmp;
		
			@tmp			 = @parts[$count+6];
			@parts[$count+6] = @parts[$count+2];
			@parts[$count+2] = @tmp;
		
			@tmp			 = @parts[$count+3];
			@parts[$count+3] = @parts[$count+5];
			@parts[$count+5] = @tmp;
		}
	
		foreach (@parts) {
			if((($counter) % 20) == 0) {
				print DEST_KEY "\n ";
			} else {
				if($counter != 0){
					print DEST_KEY " ";
				}
			}
		
			print DEST_KEY $_;
		
			$counter++;
		}
	} 


close(DEST_BIN);
close(DEST_KEY);
close(SRC);