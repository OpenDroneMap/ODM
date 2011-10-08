#!/usr/local/bin/perl

$filename_base	= $ARGV[0];
                           
$write_binary	= 1;    
         
$filename_src	= $filename_base.".key.sift";
$filename_dest	= $filename_base.($write_binary ? ".key.bin" : ".key");
$filename_image	= $filename_base.".jpg";   

open (DEST, ">$filename_dest");
open (SRC, "$filename_src");

$resolution_line = `jhead $filename_image | grep "Resolution"`;
($res_x, $res_y) = $resolution_line =~ /: ([0-9]*) x ([0-9]*)/;

$linecount = 0;
$linecount += tr/\n/\n/ while sysread(SRC, $_, 2 ** 16);

seek(SRC, 0, 0);
                                
printf ("found %d features in %s (%d x %d)\n", $linecount, $filename_image, $res_x, $res_y);
if($write_binary){              
	print DEST pack("L", $linecount);

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
		
		print DEST pack("f4 C128", @parts);
	}
} else {    
	print DEST $linecount, " 128\n";  

	while ($record = <SRC>) {
		@parts = split(/ /, $record);
	
		$counter = 0;
	
		if(@parts[3] > 3.141){
			@parts[3] -= 6.282;
		}
	         	
		@parts[3] *= -1;              
	
		printf (DEST "%.3f %.3f %.3f %.3f", @parts[1], @parts[0], @parts[2], @parts[3]);
	
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
				print DEST "\n ";
			} else {
				if($counter != 0){
					print DEST " ";
				}
			}
		
			print DEST $_;
		
			$counter++;
		}
	} 
}    

close(DEST);
close(SRC);