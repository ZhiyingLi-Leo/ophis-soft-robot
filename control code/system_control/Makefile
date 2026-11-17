CXX := clang++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -Werror -O2 -Iinclude -pthread
BUILD_DIR ?= build
CORE := src/config.cpp src/protocol.cpp src/serial.cpp src/discovery.cpp src/system.cpp
HOST := $(CORE) src/main.cpp src/keyboard.cpp src/gamepad.mm
FRAMEWORKS := -framework Foundation -framework GameController -framework IOKit

.PHONY: all demo test
all: $(BUILD_DIR)/robot_control
$(BUILD_DIR):
	mkdir -p "$@"
$(BUILD_DIR)/robot_control: $(HOST) $(wildcard include/*.hpp) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -fobjc-arc $(HOST) $(FRAMEWORKS) -o "$@"
$(BUILD_DIR)/control_test: $(CORE) tests/control_test.cpp $(wildcard include/*.hpp) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(CORE) tests/control_test.cpp -o "$@"
demo: all
	"$(BUILD_DIR)/robot_control" --mode demo
$(BUILD_DIR)/discovery_test: $(CORE) tests/discovery_test.cpp $(wildcard include/*.hpp) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(CORE) tests/discovery_test.cpp -o "$@"
test: all $(BUILD_DIR)/control_test $(BUILD_DIR)/discovery_test
	"$(BUILD_DIR)/control_test"
	PYTHONDONTWRITEBYTECODE=1 python3 tests/integration.py "$(BUILD_DIR)/robot_control"
