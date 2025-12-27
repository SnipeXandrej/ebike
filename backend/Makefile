CXX = clang++
MAKEFLAGS += -j$(shell nproc)

EXEC = ebike-backend
SOURCES = main.cpp
SOURCES += server.cpp utils.cpp myUart.cpp ads1115.cpp rollingRangeEstimation.cpp ../timer.cpp ../valueTransition.cpp
SOURCES += VescUart/VescUart.cpp VescUart/crc.cpp VescUart/buffer.cpp

OBJS = $(addsuffix .o, $(basename $(notdir $(SOURCES))))
CXXFLAGS = -std=c++23 -g -Wall -Wextra

LIBS = -lwiringPi

.PHONY: all clean notrpi

## BUILD RULES
%.o:%.cpp
		$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o:VescUart/%.cpp
		$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o: ../%.cpp
		$(CXX) $(CXXFLAGS) -c -o $@ $<

all: $(EXEC)
		@echo Build complete

## NOT_RPI BUILD (adds #define NOT_RPI)
notrpi: CXXFLAGS += -DNOT_RPI
notrpi: $(EXEC)
	@echo Build complete for NOT_RPI

$(EXEC): $(OBJS)
		$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

clean:
		rm -f $(EXEC) $(OBJS)
