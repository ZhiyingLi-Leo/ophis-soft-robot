CXX := clang++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -O2 -Iinclude
SOURCES := src/main.cpp src/serial_port.cpp src/simulated_input.cpp src/macos_gamepad.mm
.PHONY: all demo
all: bin/pcb_mac_control
bin/pcb_mac_control: $(SOURCES) $(wildcard include/*.hpp)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(SOURCES) -framework GameController -framework Foundation -framework IOKit -o $@
demo: all
	./bin/pcb_mac_control --mode simulate --demo
