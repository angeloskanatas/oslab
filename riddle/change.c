#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <string.h>

int main(int argc, char **argv)
{
	char c;
	char path[]="/tmp/riddle-";
	char str[17];

	pid_t pid;

        if((pid=fork())<0) {
                perror("fork");
                exit(1);
        }

        if(pid==0){
                char pathname[]="./riddle";
                char *argv[]={pathname, NULL};
                char *envp[]={NULL};
                execve(pathname, argv, envp);
        }

	scanf("%s %c", str, &c);
	strcat(path, str);

	int fd;
	fd=open(path, O_RDWR, S_IRUSR | S_IWUSR);
        if(fd==-1) {
                perror("open");
                exit(1);
        }

	char *addr;
	addr=mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);

	addr=addr+0x6f;
	*addr=c;

	return 0;
}
