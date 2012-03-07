LIBRARIES=

all:	$(HOME)/bin/cs-rm	$(LIBRARIES)
	(cd	build/gcc ; make)

$(HOME)/bin/cs-rm:
	mkdir	-p	$(HOME)/bin
	ln	-s	/bin/rm	~/bin/cs-rm

