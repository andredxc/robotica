CXX = g++
CFLAGS = -g -O2 -DDEBUG -fPIC -std=c++0x #-Wall

ARIA_INCLUDE=-I/usr/local/Aria/include
ARIA_LINK=-L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt

LFLAGS = $(ARIA_LINK) -lglut -lGL -lfreeimage

OBJS = Utils.o Grid.o GlutClass.o PioneerBase.o Robot.o main.o

MKDIR_P = mkdir -p
PREFIX_OBJS = $(patsubst %.o,${OBJ_DIR}/%.o,$(OBJS))
OBJ_DIR=./obj

EXEC = trab

all: $(EXEC)

%.o: src/%.cpp
	@echo "Compilando $@"
	@$(CXX) $(CFLAGS) $(ARIA_INCLUDE) -c $< -o ${OBJ_DIR}/$@

$(EXEC): $(OBJS)
	@echo "\nLinkando $(EXEC)\n"
	@$(CXX) -o $(EXEC) $(PREFIX_OBJS) $(LFLAGS)

clean:
	@echo "Limpando..."
	@rm -f $(PREFIX_OBJS) $(EXEC) *~
	@rm -f $(OBJ_DIR)/*
