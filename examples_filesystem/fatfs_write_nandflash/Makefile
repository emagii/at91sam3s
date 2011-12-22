all:	firmware.inc

firmware.inc:	mkfw
	./mkfw	> $@

mkfw:	mkfw.c
	${CC}	-o $@	mkfw.c

test:	firmware.c
	${CC}	-c	firmware.c

i:	application.c
	cp	application.c	test.c
	indent -kr	-i8 test.c

