#!/usr/bin/env perl

use strict;

use File::Basename qw( dirname );
use File::Spec;
use JSON;

my $dir = dirname($0);
my $ccd_defs = File::Spec->catfile($dir, 'ccd_defs.json');

open my $fh, $ccd_defs
    or die "Unable to open $ccd_defs : $!";
local $/;
my $json = <$fh>; # Slurp
close $fh;

decode_json($json);
print "CCD_DEFS compiles OK\n";
exit 0;
