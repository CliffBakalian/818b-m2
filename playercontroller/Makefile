CXXFLAGS = `pkg-config --cflags playerc++` -g
CFLAGS = `pkg-config --cflags playerc` -g
LDLIBS = `pkg-config --libs playerc++`
CC = g++

all: our our-na

our: our.c
	gcc -o our `pkg-config --cflags playerc` our.c `pkg-config --libs playerc`

our-na: our-na.c
	gcc -o our-na `pkg-config --cflags playerc` our-na.c `pkg-config --libs playerc`
