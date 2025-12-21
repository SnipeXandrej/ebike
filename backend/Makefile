CXX = g++
MAKEFLAGS += -j$(shell nproc)

EXEC = ebike-backend
SOURCES = main.cpp
SOURCES += server.cpp utils.cpp myUart.cpp ads1115.cpp ../timer.cpp ../valueTransition.cpp
SOURCES += VescUart/VescUart.cpp VescUart/crc.cpp VescUart/buffer.cpp

OBJS = $(addsuffix .o, $(basename $(notdir $(SOURCES))))
CXXFLAGS = -std=c++23 -g -Wall -Wextra

LIBS = -lwiringPi

#LIBS	=  `pkg-config wiringPi --libs`
#CXXFLAGS+= `pkg-config wiringPi --cflags`

## BUILD RULES
%.o:%.cpp
		$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o:VescUart/%.cpp
		$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o: ../%.cpp
		$(CXX) $(CXXFLAGS) -c -o $@ $<

all: $(EXEC)
		@echo Build complete

$(EXEC): $(OBJS)
		$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

clean:
		rm -f $(EXEC) $(OBJS)
