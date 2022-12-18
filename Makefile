MAKE = mingw32-make
CC = gcc

OBJECTS = cio.o cmarker.o fdctflt.o rdbmp.o cjpeg.o

all: $(OBJECTS)
	$(CC) -o cjpeg $(OBJECTS)

clean:
	del -f *.o cjpeg.exe cjpeg
#	rm -f *.o cjpeg.exe cjpeg

cio.o: cjpeg.h cio.h cio.c
cmarker.o: cjpeg.h cmarker.c
fdctflt.o: cjpeg.h fdctflt.c
rdbmp.o: cjpeg.h rdbmp.c
cjpeg.o: cjpeg.h cio.h cjpeg.c
