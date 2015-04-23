# replace this with the top of your X11 tree
# X11 = /exp/rcf/share/X11R5
X11 = /usr/X11R6

############## do not change below this line ####################

ROGERINCDIR = ./include

XINCDIR = $(X11)/include
XLIBDIR = $(X11)/lib 
EDLAB_XINCDIR = /usr/include/X11
EDLAB_XLIBDIR = /usr/lib/i386-linux-gnu
HSINCDIR = HSinclude
HSLIBDIR = HSlib

XAWLIB = -lXaw
XMULIB = -lXmu
XTOOLLIB = -lXt
XLIB = -lX11
XEXTLIB = -lXext
MATHLIB = -lm

LIBS =  -L$(XLIBDIR) $(XAWLIB) $(XMULIB) $(XTOOLLIB) $(XLIB) $(XEXTLIB) \
	$(MATHLIB) -L$(HSLIBDIR) $(shell ls $(HSLIBDIR)/ | sed "s/^/-l:/" | sed "s/*//" | tr '\n' ' ')

RM = rm -f
CC = ghc
CCFLAGS = -dynamic -no-hs-main -I. -I$(XINCDIR) -I$(EDLAB_XINCDIR) -I$(ROGERINCDIR) -I$(HSINCDIR)

############## do not change above this line ####################

PROG1 = x

PROG2 = roger

PROJECTOFILES = project1-MotorUnits/project1.o \
                project2-ArmKinematics/project2.o \
		project3-Vision/project3.o \
		project4-SearchTrack/project4.o \
                project4-SearchTrack/sampling.o \
		project5-StereoKinematics/project5.o \
		project6-Kalman/project6.o \
		project6-Kalman/kalman.o \
		project6-Kalman/gauss_noise.o \
                project7-ChasePunch/project7.o \
		project8-PathPlanning/project8.o \
		project8-PathPlanning/velocity_control.o \
		project9-PONG/project9.o \
		project9-PONG/update.o \
		matrix_math.o

all:  $(PROG2)

$(PROG1):
	$(CC) $(CCFLAGS) -o $@ lib/simulator.a $(LIBS) -optl-Wl,-rpath,'$$ORIGIN/$(HSLIBDIR)' project-stubs.c update.c

$(PROG2):
	$(CC) $(CCFLAGS) -o $@ lib/SocketComm.a $(LIBS) -optl-Wl,-rpath,'$$ORIGIN/$(HSLIBDIR)' project-stubs.c update.c
clean:
	$(RM) $(PROJECTOFILES) $(PROG1) $(PROG2) *~


