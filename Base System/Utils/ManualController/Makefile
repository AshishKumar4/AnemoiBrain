OBJ:=objs
CC:=clang
CPP:=clang++
LD:=ld
DRONESRC:=../../Drone
SERIALRX:= Indirect

libs:= *.so

OBJ_DIR:=./
CPP_OBJ_FILES := $(patsubst ./%.cpp, $(OBJ_DIR)/%.o,$(wildcard ./*.cpp)) \
			$(patsubst ./$(SERIALRX)/%.cpp,./$(SERIALRX)/%.o,$(wildcard ./$(SERIALRX)/*.cpp)) 
obj = $(CPP_OBJ_FILES) #$(libs)

MAIN:=ManualController.o
OUTPUT:=ManualController

INCLUDED:= -I$(DRONESRC)/include	#-I$(LIBRARY) -I$(SOURCE)

CXXFLAGS:= -w -lstdc++ -O2 -std=gnu++11 -Wall -Wextra -g -D_GLIBCXX_RELEASE -fPIC $(INCLUDED) 
LDFLAGS+=-lutil -lpthread -lDrone -lrpc
LIBRARIES:= -L$(DRONESRC)/lib/

all: clean build clean 

build: $(obj)
	$(CPP) -o $(OUTPUT) $^ $(LIBRARIES) $(LDFLAGS)
	make clean
	chmod +x $(OUTPUT)

run:
	./$(OUTPUT)

clean:
	rm -f build-crt0 *.o */*.o */*/*.o
	rm -rfv objs

%.o: %.cpp
	$(CPP) -c $< -o $@ $(CXXFLAGS)
