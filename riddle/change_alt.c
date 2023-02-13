#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>

int main(int argc, char *argv[])
{
	char c=*argv[2];
	char path[]="/tmp/riddle-";
	int fd;
	off_t offset;

	strcat(path, argv[1]);

	fd=open(path, O_RDWR, S_IRUSR | S_IWUSR);
	if(fd==-1) {
		perror("open");
		exit(1);
	}

	offset=lseek(fd, 0x6f, SEEK_SET);
	if(offset==-1) {
		perror("lseek");
		exit(1);
	}
	if(write(fd, &c, sizeof(c))!=sizeof(c)) {
		perror("write");
		exit(1);
	}

        return 0;
}
