#pragma once

#include <cstdint>
#include <cstdlib>

using uint8_t = std::uint8_t;

constexpr uint8_t LOW = 0;
constexpr uint8_t HIGH = 1;
constexpr uint8_t OUTPUT = 1;

inline uint8_t arduino_stub_pin_modes[64]{};
inline uint8_t arduino_stub_pin_states[64]{};
inline unsigned long arduino_stub_micros = 0;

inline void pinMode(uint8_t pin, uint8_t mode) { arduino_stub_pin_modes[pin] = mode; }
inline void digitalWrite(uint8_t pin, uint8_t value) { arduino_stub_pin_states[pin] = value; }
inline unsigned long micros() {
    return ++arduino_stub_micros;
}
inline long constrain(long value, long minimum, long maximum) {
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

struct SerialStub {
    void begin(long) {}
    int available() { return 0; }
    int read() { return -1; }
    void print(const char *) {}
    void print(long) {}
    void println(const char *) {}
    void println(long) {}
};

extern SerialStub Serial;
