# replace this with the top of your X11 tree
# X11 = /exp/rcf/share/X11R5
X11 = /usr/X11R6

############## do not change below this line ####################

ROGERINCDIR = ./include
HSINCDIR    = ./HSinclude

XINCDIR = $(X11)/include
XLIBDIR = $(X11)/lib
EDLAB_XINCDIR = /usr/include/X11
EDLAB_XLIBDIR = /usr/lib/i386-linux-gnu
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

PROG = x

all: $(PROG)

$(PROG):
	$(CC) $(CCFLAGS) -o $@ lib/simulator.a $(LIBS) -optl-Wl,-rpath,'$$ORIGIN/$(HSLIBDIR)' project-stubs.c

clean:
	$(RM) project-stubs.o $(PROG) *~

