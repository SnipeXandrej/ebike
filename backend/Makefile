CXX = g++

EXEC = ebike-backend
SOURCES = main.cpp

OBJS = $(addsuffix .o, $(basename $(notdir $(SOURCES))))
CXXFLAGS = -std=c++23 -g -Wall

LIBS = -lwiringPi

#LIBS	=  `pkg-config wiringPi --libs`
#CXXFLAGS+= `pkg-config wiringPi --cflags`

## BUILD RULES
%.o:%.cpp
		$(CXX) $(CXXFLAGS) -c -o $@ $<

all: $(EXEC)
		@echo Build complete

$(EXEC): $(OBJS)
		$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

clean:
		rm -f $(EXEC) $(OBJS)
