CC       := g++
FLAGS    := -O2 -Wall -g -std=c++14 -arch x86_64
INCLUDE  := -I./include -I/usr/local/include
LDLIBS   := -L/usr/local/lib -lpng -lyaml-cpp -lGLEW -lm -lpthread -framework GLUT -framework OpenGL

CCFLAGS  := $(FLAGS) $(INCLUDE)
LDFLAGS  := $(FLAGS)

BIN_PATH := bin
OBJ_PATH := obj
SRC_PATH := src

TARGET_NAME := renderer
TARGET      := $(BIN_PATH)/$(TARGET_NAME)

SRC := $(wildcard $(SRC_PATH)/*.cpp) $(wildcard $(SRC_PATH)/*.c)
OBJ := $(addprefix $(OBJ_PATH)/, $(notdir $(SRC:.cpp=.o)))
OBJ := $(OBJ:.c=.o)

CLEAN_LIST := $(TARGET) $(OBJ)

default: makedir all

$(TARGET): $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $(OBJ) $(LDLIBS)

$(OBJ_PATH)/%.o: $(SRC_PATH)/%.cpp
	$(CC) $(CCFLAGS) -c $< -o $@

$(OBJ_PATH)/%.o: $(SRC_PATH)/%.c
	$(CC) $(CCFLAGS) -c $< -o $@

.PHONY: makedir
makedir:
	mkdir -p $(BIN_PATH) $(OBJ_PATH)

.PHONY: all
all: $(TARGET)

.PHONY: clean
clean:
	rm -f $(CLEAN_LIST)