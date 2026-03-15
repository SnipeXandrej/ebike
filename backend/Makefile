CXX = clang++
MAKEFLAGS += -j$(shell nproc)

SRC_ROOT := $(abspath ..)
BUILD := build

EXEC = ebike-backend
SOURCES = main.cpp
SOURCES += server.cpp myUart.cpp rollingRangeEstimation.cpp ../timer.cpp ../valueTransition.cpp ../messagingUtils.cpp ../commonUtils.cpp ../loopRateLimiter.cpp ../rampLimiter.cpp
SOURCES += VescUart/VescUart.cpp VescUart/crc.cpp VescUart/buffer.cpp

CXXFLAGS = -std=c++23 -g -Wall -Wextra -I$(SRC_ROOT)

NORM_SOURCES := $(patsubst $(SRC_ROOT)/%,%,$(abspath $(SOURCES)))
OBJS := $(NORM_SOURCES:%.cpp=$(BUILD)/%.o)

ifeq ($(UNAME_S), Linux) #LINUX
	LIBS += -lwiringPi
    SOURCEs += ads1115.cpp
endif

.PHONY: all clean notrpi

## BUILD RULES
$(BUILD)/%.o: $(SRC_ROOT)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

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
