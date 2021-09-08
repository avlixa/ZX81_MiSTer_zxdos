#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main (int argc, char *argv[])
{
	FILE *f;
	unsigned char *scr;
	char nombre[256];
	int i,j,leido;
	
	if (argc<2)
		return 1;
	
	scr = malloc(65536);
	f = fopen (argv[1],"rb");
	if (!f)
		return 1;
		
	leido = fread (scr, 1, 65536, f);
	fclose (f);
	
	strcpy (nombre, argv[1]);
	nombre[strlen(nombre)-3]=0;
	strcat (nombre, "hex");
	
	f = fopen (nombre, "wt");
	j=1;
	for (i=0;i<leido;i++)
		if (j++ == 16) { fprintf (f, "%.2X\n", scr[i]); j=1; }
		else
			fprintf (f, "%.2X ", scr[i]);
	fclose(f);
	
	return 0;
}

