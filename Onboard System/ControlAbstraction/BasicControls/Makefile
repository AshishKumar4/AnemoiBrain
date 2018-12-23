OBJ:=objs
CC:=gcc
CPP:=g++
LD:=ld
AS:=as
NASM:=nasm

AIRSIM_AIRLIB_DIR:=/home/unrealengine/Desktop/GuardianGrounds/Plugins/AirSim/Source/AirLib/include/vehicles/multirotor
AIRSIM_PROJ_SOURCE_DIR:=/home/unrealengine/Desktop/GuardianGrounds/Plugins/AirSim/Source
AIRSIM_INCLUDE:=-I$(AIRSIM_PROJ_SOURCE_DIR)/AirLib/deps/rpclib/include -I$(AIRSIM_AIRLIB_DIR) -I$(AIRSIM_PROJ_SOURCE_DIR)/AirLib/deps/eigen3 -I$(AIRSIM_PROJ_SOURCE_DIR)/AirLib/include

OBJ_DIR:=./
C_OBJ_FILES := $(patsubst ./%.c,$(OBJ_DIR)/%.o,$(wildcard ./*.c))	\
			$(patsubst ./SPI/%.c,./SPI/%.o,$(wildcard ./SPI/*.c))
CPP_OBJ_FILES := $(patsubst ./%.cpp, $(OBJ_DIR)/%.o,$(wildcard ./*.cpp)) \
			$(patsubst ./SPI/%.cpp,./SPI/%.o,$(wildcard ./SPI/*.cpp)) 
obj = $(CPP_OBJ_FILES) $(C_OBJ_FILES)

MAIN:=ControlServer.o
OUTPUT:=ControlServer

INCLUDED:=# $(AIRSIM_INCLUDE)	#-I$(LIBRARY) -I$(SOURCE)

LIBS_RPI3:=$(wildcard ./lib/rpi3/*.so)#./lib/rpi3*.so
LIBS_RPI0:=$(wildcard ./lib/rpi0/*.so)
LIBS_PC:=$(wildcard ./lib/pc/*.so)

CXXFLAGS:= -lstdc++ -O2 -std=gnu++11 $(INCLUDED)
LDFLAGS+=-lutil -lpthread 
LD_SPL_FLAGS+=-lwiringPi
all: clean build clean 
test: clean test-build clean
pi3: clean build-pi3 clean
pi0: clean build-pi0 clean

brun: all run

build-pi3: $(obj)
	$(CXX) -o $(OUTPUT) $^ $(LIBS_RPI3) $(LDFLAGS) $(LD_SPL_FLAGS)

build-pi0: $(obj)
	$(CSS) -o $(OUTPUT) $^ $(LIBS_RPI0) $(LDFLAGS) $(LD_SPL_FLAGS)

build: $(obj)
	$(CXX) -o $(OUTPUT) $^ $(LIBS_RPI0) $(LDFLAGS) $(LD_SPL_FLAGS)

test-build: $(obj)
	$(CXX) -o $(OUTPUT) $^ $(LIBS_PC) $(LDFLAGS)

build-main:	$(MAIN)
	$(CXX) -o $(OUTPUT) $^ $(LDFLAGS)
	chmod +x $(OUTPUT)

run:
	./$(OUTPUT)

clean:
	rm -f build-crt0 *.o */*.o */*/*.o */*/*/*.o
	rm -rfv objs

%.o: %.c
	$(CC) -c $< -o $@ $(CXXFLAGS)

%.o: %.cpp
	$(CPP) -c $< -o $@ $(CXXFLAGS)
