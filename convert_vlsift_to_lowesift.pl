#!/usr/local/bin/perl

$filename = $ARGV[0];

open (DEST, ">$filename.key");
open (SRC, "$filename.key.tmp");

$resolution_line = `jhead $filename.jpg | grep "Resolution"`;
($res_x, $res_y) = $resolution_line =~ /: ([0-9]*) x ([0-9]*)/;

$linecount = 0;
$linecount += tr/\n/\n/ while sysread(SRC, $_, 2 ** 16);

seek(SRC, 0, 0);

print DEST $linecount;
print DEST " 128\n";

printf ("found %d features in %s.jpg\n\n", $linecount, $filename);

while ($record = <SRC>) {
	@parts = split(/ /, $record);
	
	$counter = 0;
	
	$parts[0] = $res_x-$parts[0];
	
	foreach (@parts) {
		
		if((($counter-4) % 20) == 0) {
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

close(DEST);
close(SRC);