#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>

int main(int argc, char *argv[])
{
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

	sleep(1); //wait for riddle to start (sync)
        if(kill(pid, SIGCONT)<0) {
		perror("kill");
		exit(1);
	}

        if(wait(NULL)==-1) {
                perror("wait");
                exit(1);
        }

        return 0;
}
