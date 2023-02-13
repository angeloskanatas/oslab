/*
 * fork_test.c
 *
 * A userspace program to test the functionality of the
 * lunix character device driver using multiple forks.
 *
 * Angelos-Nikolaos Kanatas <el19169@mail.ntua.gr>
 * Ioannis Asprogerakas <el18942@mail.ntua.gr>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "lunix-chrdev.h"

int safe_atoi(char *s, int *val)
{
	long l;
	char *endp;

	l = strtol(s, &endp, 10);
	if (s != endp && *endp == '\0') {
		*val = l;
		return 0;
	} else 
		return -1;
}

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
	int procnt;

	if ((argc != 3) || (safe_atoi(argv[2], &procnt) < 0) || (procnt <=0)) {
		fprintf(stderr, "Usage: %s <lunix-chrdev> <number of processes>\n\n", argv[0]);
		exit(1);
	}

	int fd;
	fd = open(argv[1], O_RDONLY);
	if (fd == -1) {
		perror("open");
		exit(1);
	}

	pid_t pid;
	for(int i=0; i<procnt; i++) {
		pid = fork();
		if (pid < 0) {
			perror("fork");
			exit(1);
		}
		if (pid == 0) {
			char buff[LUNIX_CHRDEV_BUFSZ];
			char output[30];
			int output_size;
			for(;;) {
				if (read(fd, buff, sizeof(buff)) == -1) {
					perror("read");
					close(fd);
					exit(1);
				}
				output_size = snprintf(output, 30, "[PID %d]:%s\n", getpid(), buff);
				insist_write(1, output, (size_t) output_size);
			}
		}
	}

	for(int i=0; i<procnt; i++) {
		if ((pid = wait(NULL)) == -1) {
			perror("wait");
			exit(1);
		}
	}

	/* Unreachable */
	return 100;
}