/*
 * mmap_test.c
 *
 * A userspace program to test the functionality of the
 * lunix character device driver using mmap() (memory-mapped I/O).
 *
 * Angelos-Nikolaos Kanatas <el19169@mail.ntua.gr>
 * Ioannis Asprogerakas <el18942@mail.ntua.gr>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#include "lunix.h"

int main(int argc, char **argv)
{
	struct lunix_msr_data_struct *addr;
	uint32_t measurement;
	uint32_t buf_timestamp = 0;
	size_t len;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <lunix-chrdev>\n\n", argv[0]);
		exit(1);
	}

	if ((len = sysconf(_SC_PAGESIZE)) == -1) {
		perror("sysconf(_SC_PAGESIZE)");
		exit(1);
	}

	int fd;
	fd = open(argv[1], O_RDONLY);
	if (fd == -1) {
		perror("open");
		exit(1);
	}

	addr = mmap(NULL, len, PROT_READ, MAP_PRIVATE, fd, 0);
	if (addr == MAP_FAILED) {
		perror("mmap");
		exit(1);
	}

	while(1) {
		if (buf_timestamp != addr->last_update) {
			buf_timestamp = addr->last_update;
			measurement = addr->values[0];
			printf("%u\n", measurement);
		}
		/* FIXME */
		usleep(200000); //sleep until new data arrives
	}

	/* FIXME: convert raw data to floating point values specific to the measurement */
	
	/* Unreachable */
	return 100;
}
