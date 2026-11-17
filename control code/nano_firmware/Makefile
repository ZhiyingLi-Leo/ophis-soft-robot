CXX := clang++
.PHONY: test clean
test: build/arduino_syntax_test
	./build/arduino_syntax_test
build/arduino_syntax_test: tests/arduino_syntax_test.cpp tests/arduino_stub/Arduino.h arduino/propulsion_controller/propulsion_controller.ino
	@mkdir -p build
	$(CXX) -std=c++17 -Wall -Wextra -Wpedantic -Itests tests/arduino_syntax_test.cpp -o $@
clean:
	rm -f build/arduino_syntax_test
