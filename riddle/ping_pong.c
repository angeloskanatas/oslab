#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

int main(void)
{
	int pipefd1[2];
	int pipefd2[2];

	if(pipe(pipefd1)==-1) {
		perror("pipe1");
		exit(1);
	}

	if(pipe(pipefd2)==-1) {
		perror("pipe2");
		exit(1);
	}

	dup2(pipefd1[0], 33);
	dup2(pipefd1[1], 34);
	dup2(pipefd2[0], 53);
	dup2(pipefd2[1], 54);

	char pathname[]="./riddle";
        char *argv[]={pathname, NULL};
        char *envp[]={NULL};
        execve(pathname, argv, envp);

}
