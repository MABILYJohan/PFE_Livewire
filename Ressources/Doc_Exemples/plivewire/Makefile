# Makefile for Intelligent Snake
# Steven Marx, University of Rochester

FILES = glutMaster.cc glutWindow.cc image.cc isnake.cc liveWire.cc main.cc mainWindow.cc point.cc pq.cc contour.cc displayWindow.cc

OBJECTS = glutMaster.o glutWindow.o image.o isnake.o liveWire.o main.o mainWindow.o point.o pq.o contour.o displayWindow.o

EXECFIL = isnake
LIBS = -lglut -lGLU -lGL -lXmu -lXext -lX11 -lm

# Installation at UR CS
FLAGS = -I/u/smarx/sparc_solaris/glut-3.7/include -L/u/smarx/sparc_solaris/glut-3.7/lib/glut

$(EXECFIL): $(OBJECTS)
	gcc $(FLAGS) $(OBJECTS) -o $(EXECFIL) $(LIBS)

.cc.o :
	gcc $(FLAGS) -c $<
