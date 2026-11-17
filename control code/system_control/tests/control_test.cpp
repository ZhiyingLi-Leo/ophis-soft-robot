#include "robot.hpp"
#include <cassert>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>
#include <unistd.h>

namespace {
std::vector<unsigned char> reply(int id, int op = 0x30, int address = 0) {
    std::vector<unsigned char> f{0xaa, 0x55, 15, static_cast<unsigned char>(id), static_cast<unsigned char>(op),
        static_cast<unsigned char>(address), 0, 0xe8, 3, 0xd0, 7, 0, 0, 0, 0, 0, 0, 25, 0};
    unsigned sum = 0; for (size_t i = 2; i < f.size(); ++i) sum += f[i];
    f.push_back(sum & 255); return f;
}
template<class F> void rejects(F function) {
    bool caught = false;
    try { function(); } catch (const std::exception&) { caught = true; }
    assert(caught);
}
void protocol() {
    using namespace robot::inspire;
    assert(command(1, 0x30) == (std::vector<unsigned char>{0x55, 0xaa, 1, 1, 0x30, 0x32}));
    rejects([] { command(255, 0x30); });
    auto f = reply(2); State s;
    std::vector<unsigned char> b{4, 5, 0x55, 0xaa};
    auto bad = f; bad.back() ^= 1;
    b.insert(b.end(), bad.begin(), bad.end());
    auto other = reply(1); b.insert(b.end(), other.begin(), other.end());
    assert(!extract(b, 2, 0x30, 0, s));
    for (size_t i = 0; i < f.size(); ++i) {
        b.push_back(f[i]); bool found = extract(b, 2, 0x30, 0, s);
        assert(found == (i + 1 == f.size()));
    }
    assert(s.position == 2000 && s.target == 1000 && s.temperature == 25);
    b = reply(2, 0x32, 0x25);
    assert(!extract(b, 2, 0x30, 0, s));
    assert(extract(b, 2, 0x32, 0x25, s));
    b.assign(10000, 1); assert(!extract(b, 2, 0x30, 0, s)); assert(b.size() == 19);
}
void limits() {
    using namespace robot;
    auto c = load_config(""); validate(c);
    assert(valid_feedback(-0.04, c.cylinders[0], 0.05));
    assert(!valid_feedback(-0.056, c.cylinders[0], 0.05));
    assert(!valid_feedback(16.008, c.cylinders[0], 0.05));
    rejects([&] { position_code(-0.04, c.cylinders[0]); });
    c.auto_zero = true; c.cylinders[0].minimum = 1; rejects([&] { validate(c); });
    c.cylinders[0].minimum = 0; validate(c);
    c.cylinders[1].id = 1; rejects([&] { validate(c); });
    c = load_config(""); c.cylinders[0].speed = 0.001; rejects([&] { validate(c); });
    c = load_config(""); c.deadzone = std::numeric_limits<double>::quiet_NaN(); rejects([&] { validate(c); });
    CylinderConfig x{1, 16, 0.003, 5.003, 0.5};
    assert(position_code(x.minimum, x) == 1);
    assert(position_code(x.maximum, x) == 625);
    rejects([&] { position_code(-1, x); });
    rejects([] { axis_value(std::numeric_limits<double>::quiet_NaN(), 0.12); });
    assert(axis_value(0.1, 0.12) == 0 && axis_value(-1, 0.12) == -1);
    assert(ramp(100, -100, 0.02, 1000) == 80);
}
void stale_input() {
    using namespace robot;
    auto c = load_config("");
    System system(c);
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    auto before = system.bending_status();
    assert(before.ready && system.propulsion_status().ready);
    // Independent workers move together, but stop if the main input loop stalls.
    for (int i = 0; i < 20; ++i) {
        system.submit({true, 0.5, {0.8, -0.6, 0.4}, Clock::now()});
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    assert(system.propulsion_status().speed > 0);
    auto moving = system.bending_status();
    assert(moving.position[0] > before.position[0] && moving.position[1] < before.position[1]);
    // A stop immediately replaced by new input must still reach the worker.
    system.submit({});
    system.submit({true, 0.5, {0.8, -0.6, 0.4}, Clock::now()});
    bool observed_stop = false;
    for (int i = 0; i < 30; ++i) {
        if (system.propulsion_status().speed == 0) observed_stop = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    assert(observed_stop);
    std::this_thread::sleep_for(std::chrono::milliseconds(350));
    assert(system.propulsion_status().speed == 0);
    auto stopped = system.bending_status();
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    assert(system.bending_status().position == stopped.position);
    system.shutdown();
    assert(system.propulsion_status().error.empty() && system.bending_status().error.empty());
}
}
int main() {
    protocol(); limits(); stale_input();
    std::cout << "PASS: fragmented/invalid frames, limits, simultaneous control, stale-input stop\n";
}
