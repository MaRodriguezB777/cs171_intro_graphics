CC = g++
FLAGS = -g -std=c++11 -DGL_SILENCE_DEPRECATION -fsanitize=address

INCLUDE = -I../ -I/usr/X11R6/include -I/usr/include -I/opt/homebrew/include
LIBDIR = -L/usr/X11R6/lib -L/usr/local/lib -L/opt/homebrew/lib
SOURCES = *.cpp
LIBS = -lGLEW -lm -framework GLUT -framework OpenGL

EXENAME = keyframe

ibar: $(SOURCES)
	$(CC) $(FLAGS) -o $(EXENAME) $(INCLUDE) $(LIBDIR) $(SOURCES) $(LIBS)

clean:
	rm -f *.o $(EXENAME)

.PHONY: all
all: clean ibar

