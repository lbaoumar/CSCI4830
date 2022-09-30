#include <stdio.h>
#include <stdlib.h>
#include <string.h> // For strtok() and strcmp()
#include <unistd.h> // For fork(), pid_t
#include <sys/types.h>


void prompt()
{
	printf("@> ");
}

void read_command(char cmd[], char *par[])
{
	char line[4095];
	int count = 0, i = 0, j = 0;
	char *array[100], *pch;

	//read line
	for( ;; )
	{
		int c = fgetc(stdin);
		line[count++] = (char) c;
		if(c == '\n') break;
	}
	if(count == 1) return;
	pch = strtok(line, " \n");

	//parse line
	while(pch != NULL)
	{
		array[i++] = strdup (pch);
		pch = strtok (NULL, " \n");
	}

	strcpy (cmd, array[0]);	//first word in the command

	for(int j = 0; j < i; j++)
	{
		par[j] = array[j];
		par[i] = NULL;		//to finish the parameter list
	}


}

int main()
{	
	//array of char and pointer
	char cmd[100], command[100], *parameter[20];
	char *envp[] = {(char *) "PATH=/bin", 0};
	while(1)
	{
		prompt();
		read_command(command, parameter);
		if(fork() != 0)
			wait(NULL);
	}
	return 0;
}

