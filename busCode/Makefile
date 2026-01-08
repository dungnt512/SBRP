# Makefile for School Bus Routing Problem Algorithm

EXEC=solver

HEADS=bpp.h fns.h initsol.h input.h main.h mobj.h optimiser.h setcover.h

OBJ=bpp.o fns.o initsol.o input.o main.o mobj.o optimiser.o setcover.o

CPP=g++
OPTS=-O3 -Wall ${GFLAGS}

all: ${EXEC}

${EXEC}: ${OBJ}
	${CPP} ${OPTS} -o $@ ${OBJ}

%.o: %.cpp ${HEADS}
	${CPP} ${OPTS} -c -o $@ $<

clean:
	rm -f ${OBJ} ${EXEC}

