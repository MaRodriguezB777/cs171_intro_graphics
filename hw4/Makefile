CC = g++
FLAGS = -g -std=c++11 -DGL_SILENCE_DEPRECATION

INCLUDE = -I../ -I/usr/X11R6/include -I/usr/include/GL -I/usr/include -I/opt/homebrew/include
LIBDIR = -L/usr/X11R6/lib -L/usr/local/lib -L/opt/homebrew/lib
SOURCES = *.cpp
LIBS = -lGLEW -lm -lpng -framework GLUT -framework OpenGL

EXENAME = glslRenderer

glslRenderer: $(SOURCES)
	$(CC) $(FLAGS) -o $(EXENAME) $(INCLUDE) $(LIBDIR) $(SOURCES) $(LIBS)

.PHONY: run
run:
	./$(EXENAME)

.PHONY: clean
clean:
	rm -f *.o $(EXENAME) core glslRenderer

.PHONY: all
all: clean glslRenderer
