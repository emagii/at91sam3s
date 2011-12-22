#include	<stdio.h>

void	mkline (char c)
{
	int	i;
	putchar('\t');
	for (i = 0; i < 16; i++) {
		printf("0x%02x,%c", c, (i==15)?'\n':' ');
	}
}


void	mkpage (int page)
{
	int	i;

	printf ("/* Page = %d */\n", page);
	for (i = 0; i < 16; i++) {
		mkline ((char) i);
	}
}

unsigned int main (void)
{
	int	page;

	for (page = 0; page < 140*4; page++ ) {
		mkpage (page);
	}
	return	0;
}

