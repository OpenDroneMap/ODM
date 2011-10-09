/* 
 *  Copyright (c) 2008-2010  Noah Snavely (snavely (at) cs.cornell.edu)
 *    and the University of Washington
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

/* KeyMatch.cpp */
/* Read in keys, match, write results to a file */

#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include "keys2a.h"

int main(int argc, char **argv) {
    char *keys1_in;
    char *keys2_in;
    char *file_out;
    double ratio, threshold;
    
    if (argc != 6) {
		printf("Usage: %s <keys1.in> <keys2.in> <out.txt> <ratio> <threshold>\n", argv[0]);
		return -1;
    }
    
    keys1_in = argv[1];
    keys2_in = argv[2];
    file_out = argv[3];

    ratio 		= atof(argv[4]);
    threshold 	= atof(argv[5]);

    clock_t start = clock();

    unsigned char *keys1, *keys2;

    int num1 = ReadKeyFile(keys1_in, &keys1);
    int num2 = ReadKeyFile(keys2_in, &keys2);


    /* Compute likely matches between two sets of keypoints */
    std::vector<KeypointMatch> matches = 
	MatchKeys(num1, keys1, num2, keys2, ratio);

#if 0
    std::vector<KeypointMatch> matches_sym = 
	MatchKeys(num2, keys2, num1, keys1);
#endif

    int num_matches = (int) matches.size();
    // int num_matches_sym = (int) matches_sym.size();

 //   printf("num_matches = %d\n", num_matches);
    // printf("num_matches_sym = %d\n", num_matches_sym);

#if 0
    /* Prune asymmetric matches */
    for (int i = 0; i < num_matches; i++) {
	int idx1 = matches[i].m_idx1;
	int idx2 = matches[i].m_idx2;
	
	for (int j = 0; j < num_matches_sym; j++) {
	    if (matches_sym[j].m_idx1 == idx2) {
		if (matches_sym[j].m_idx2 != idx1) {
		    matches.erase(matches.begin() + i);
		    i--;
		    num_matches--;
		}

		break;
	    }
	}
    }
#endif

    clock_t end = clock();    

	int m = (num1 < num2 ? num1 : num2);
	float r = ((float)num_matches * 100 / m);

	bool used = false;
	
    if (num_matches >= 16 && r > threshold) {
		used = true;
	
		FILE *f = fopen(file_out, "w");
		
		/* Write the number of matches */
		fprintf(f, "%d\n", (int) matches.size());

		for (int i = 0; i < num_matches; i++) {
		    fprintf(f, "%d %d\n", matches[i].m_idx1, matches[i].m_idx2);
		}

		fclose(f);
    }

	
  if(used) printf("\n%8d matches (%4.1f%%) took %5.2fs for %s\t", 
		num_matches,
		r,
	   (end - start) / ((double) CLOCKS_PER_SEC),
		file_out);

		
/*    printf("%8d matches = %5.2f%% %d in %6.3fs in %s\n", 
		num_matches,
		r,
		m,
	   (end - start) / ((double) CLOCKS_PER_SEC),
		file_out);
		*/
}
