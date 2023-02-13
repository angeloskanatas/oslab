/*
 * ioctl_test.c
 * 
 * A userspace program to test the use of ioctl
 * syscall and set the mode of data tranferring from
 * the character device.
 *
 * Angelos-Nikolaos Kanatas <el19169@mail.ntua.gr>
 * Ioannis Asprogerakas <el18942@mail.ntua.gr>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>

#include "lunix-chrdev.h"

ssize_t insist_write(int fd, const char *buf, size_t count)
{
	ssize_t ret;
	size_t orig_count = count;

	while (count > 0) {
		ret = write(fd, buf, count);
		if (ret < 0)
			return ret;
		buf += ret;
		count -= ret;
	}
	return orig_count;
}

int main(int argc, char **argv)
{
	if ((argc != 3) || (strcmp(argv[2], "raw") && strcmp(argv[2], "cooked"))) {
		fprintf(stderr, "Usage: %s <lunix-chrdev> <raw/cooked>\n\n", argv[0]);
		exit(1);
	}

	int fd;
	fd = open(argv[1], O_RDONLY);
	if (fd == -1) {
		perror("open");
		exit(1);
	}

	if (!strcmp(argv[2], "raw")) {
		if (ioctl(fd, LUNIX_IOC_RAWDATA) == -1) {
			perror("ioctl");
			exit(1);
		}
	}

	ssize_t rcnt;
	char buff[LUNIX_CHRDEV_BUFSZ];

	for(;;) {
		rcnt = read(fd, buff, sizeof(buff));
		if (rcnt == -1) {
			perror("read");
			close(fd);
			exit(1);
		}
		insist_write(1, buff, rcnt);
	}

	/* Unreachable */
	return 100;
}	