#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>

int main(void)
{
        pid_t pid=getpid();
	char pathname[]="./riddle";
        char *argv[]={pathname, NULL};
        char *envp[]={NULL};

	if(pid==32767)
		execve(pathname, argv, envp);

	while(pid!=32767) {
		if((pid=fork())<0) {
                	perror("fork");
                	exit(1);
        	}
		if(pid==0) {
			if(getpid()==32767)
				execve(pathname, argv, envp);
			else
				exit(0);
		}
	}

        return 0;
}
