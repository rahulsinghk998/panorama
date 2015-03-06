# Makefile for project 2

PROJ3=CSC5280
PROJ3_OBJS=BlendImages.o jacob.o P3Math.o \
		FeatureSet.o FeatureAlign.o GlobalAlign.o \
		WarpSpherical.o Project3.o

IMAGELIB=ImageLib/libImage.a

CC=g++
CPPFLAGS=-Wall -O3 -I/uns/include
LIB_PATH=-L/uns/lib -L/usr/X11R6/lib

all: $(PROJ3)

$(IMAGELIB): 
	make -C ImageLib

$(PROJ3): $(PROJ3_OBJS) $(IMAGELIB)
	$(CC) -o $@ $(PROJ3_OBJS) $(LIB_PATH) $(IMAGELIB)

clean:
	rm -f *.o *~ $(PROJ3)
