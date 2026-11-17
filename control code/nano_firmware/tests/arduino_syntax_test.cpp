#include "arduino_stub/Arduino.h"

#include <cassert>
#include <iostream>

SerialStub Serial;

#include "../arduino/propulsion_controller/propulsion_controller.ino"

int main() {
    setup();

    // 上电后脉冲差分端相同，四个电机不应运动。
    assert(arduino_stub_pin_states[PULSE_MINUS_PIN] == LOW);
    assert(arduino_stub_pin_states[PULSE_PLUS_PIN] == LOW);
    assert(arduino_stub_pin_states[DIRECTION_MINUS_PIN] == LOW);

    // 正速：上下两组方向必须相反。
    setSpeed(1000);
    assert(commandedSpeed == 1000);
    assert(halfPeriodMicros == 500);
    assert(arduino_stub_pin_states[UPPER_DIRECTION_PLUS_PIN] == UPPER_FORWARD_LEVEL);
    assert(arduino_stub_pin_states[LOWER_DIRECTION_PLUS_PIN] == LOWER_FORWARD_LEVEL);
    assert(arduino_stub_pin_states[UPPER_DIRECTION_PLUS_PIN] !=
           arduino_stub_pin_states[LOWER_DIRECTION_PLUS_PIN]);

    // 等待方向建立时间和半个脉冲周期后，P+ 应翻转。
    arduino_stub_micros += DIRECTION_SETUP_MICROS + halfPeriodMicros;
    updateStepPulse();
    assert(arduino_stub_pin_states[PULSE_PLUS_PIN] == HIGH);

    // 负速：两组方向同时取反，且先将脉冲拉回静止电平。
    setSpeed(-1000);
    assert(commandedSpeed == -1000);
    assert(arduino_stub_pin_states[PULSE_PLUS_PIN] == LOW);
    assert(arduino_stub_pin_states[UPPER_DIRECTION_PLUS_PIN] == !UPPER_FORWARD_LEVEL);
    assert(arduino_stub_pin_states[LOWER_DIRECTION_PLUS_PIN] == !LOWER_FORWARD_LEVEL);
    updateStepPulse();
    assert(arduino_stub_pin_states[PULSE_PLUS_PIN] == LOW);

    // 超范围速度必须被夹到最大值。
    setSpeed(999999);
    assert(commandedSpeed == MAX_SPEED_STEPS_PER_SECOND);

    stopMotor();
    assert(commandedSpeed == 0);
    assert(arduino_stub_pin_states[PULSE_PLUS_PIN] == LOW);

    loop();
    std::cout << "arduino_pin_logic_test: PASS\n";
    return 0;
}
