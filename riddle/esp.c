#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>

ssize_t insist_write(int fd, const char *buf, size_t count)
{
	ssize_t ret;
        size_t orig_count=count;

        while(count>0) {
                ret=write(fd, buf, count);
                if(ret<0)
                	return ret;
                buf+=ret;
                count-=ret;
        }

        return orig_count;
}

int main(int argc, char *argv[])
{
        pid_t pid;
	int fd;

	fd=open("secret_number", O_CREAT | O_RDWR | O_TRUNC, S_IRUSR | S_IWUSR);
        if(fd==-1) {
                perror("open");
                exit(1);
        }

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

	sleep(2);

	ssize_t rcnt;
	char buff[4096];
	off_t offset;

	if((offset=lseek(fd, 61, SEEK_SET))==-1) {
        	perror("lseek");
                exit(1);
	}

	for(;;) {
		rcnt=read(fd, buff, sizeof(buff));
		if(rcnt==0) break;
		if(rcnt==-1) {
			perror("read");
			close(fd);
			exit(1);
		}
		insist_write(1, buff, rcnt);
	}

	close(fd);

        if(wait(NULL)==-1) {
                perror("wait");
                exit(1);
        }

        return 0;
}
