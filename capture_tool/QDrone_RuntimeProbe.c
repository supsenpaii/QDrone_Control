#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <string.h>
#include <time.h>
int main(void){
 char self[4096],path[4096];ssize_t len=readlink("/proc/self/exe",self,sizeof(self)-1);
 if(len<0){perror("readlink");return 1;}self[len]=0;char *slash=strrchr(self,'/');if(slash)*slash=0;
 snprintf(path,sizeof(path),"%s/QDrone_Capture.rt-linux_x64",self);
 printf("PROBE binary directory: %s\nCapture binary: %s\n",self,path);fflush(stdout);
 pid_t child=fork();if(child==0){execl(path,path,"-w","-d","/tmp","-uri","tcpip://192.168.2.52:17005",(char*)0);perror("exec capture");_exit(127);}
 if(child<0){perror("fork");return 1;}
 int status=0,done=0;for(int k=0;k<50;k++){if(waitpid(child,&status,WNOHANG)==child){done=1;break;}usleep(100000);}
 if(!done){printf("Capture stayed alive for 5 seconds; terminating only probe child %d.\n",(int)child);kill(child,SIGTERM);waitpid(child,&status,0);}
 if(WIFSIGNALED(status))printf("Capture child terminated by signal %d (%s)\n",WTERMSIG(status),strsignal(WTERMSIG(status)));
 else if(WIFEXITED(status))printf("Capture child exit status %d\n",WEXITSTATUS(status));
 fflush(stdout);
 FILE *kernel=popen("dmesg 2>/dev/null | tail -n 12","r");if(kernel){char line[2048];while(fgets(line,sizeof(line),kernel))fputs(line,stdout);pclose(kernel);}fflush(stdout);
 return 0;
}
