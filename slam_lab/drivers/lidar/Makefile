CXX ?= g++
SDK_ROOT ?= ../../third_party/rplidar_sdk
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -pedantic
CPPFLAGS += -I$(SDK_ROOT)/sdk/include
LDLIBS += $(SDK_ROOT)/output/Linux/Release/libsl_lidar_sdk.a -lpthread

.PHONY: all clean

all: lidar_control scan_bridge

lidar_control: lidar_control.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS)

scan_bridge: scan_bridge.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS)

clean:
	rm -f lidar_control scan_bridge
