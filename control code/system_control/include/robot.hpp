#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace robot {
using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point since);

struct CylinderConfig {
    int id = 0;
    double stroke = 0, minimum = 0, maximum = 0, speed = 0;
};
struct Config {
    std::string propulsion = "nano", propulsion_port, cylinder_port;
    int cylinder_baud = 115200, max_speed = 3200;
    double acceleration = 3200, deadzone = 0.12;
    std::array<CylinderConfig, 3> cylinders{};
    bool hardware = false;
    bool auto_zero = false;
    double zero_tolerance_mm = 0;
};
Config load_config(const std::string& path);
void validate(const Config& config);
bool valid_feedback(double mm, const CylinderConfig& cylinder, double zero_tolerance);
double axis_value(double value, double deadzone);
int ramp(int current, int target, double dt, double acceleration);
int position_code(double mm, const CylinderConfig& c);

struct Input {
    double propulsion = 0;
    std::array<double, 3> cylinders{};
    bool connected = true, enable = false, held = false, stop = false, quit = false;
    std::optional<std::array<double, 3>> position = std::nullopt;
};
class InputSource {
public:
    virtual ~InputSource() = default;
    virtual Input poll() = 0;
    virtual std::string description() const = 0;
};
std::unique_ptr<InputSource> gamepad();
std::unique_ptr<InputSource> keyboard(bool commands = false);

// A single input snapshot replaces the previous intent; commands are not queued.
struct Intent {
    bool active = false;
    double propulsion = 0;
    std::array<double, 3> cylinders{};
    Clock::time_point time = Clock::now();
    std::optional<std::array<double, 3>> position = std::nullopt;
};
struct Status {
    bool ready = false;
    std::string error;
    int speed = 0;
    std::array<double, 3> position{}, target{};
    std::array<Clock::time_point, 3> feedback_time{};
    Clock::time_point time = Clock::now();
};

class System {
public:
    explicit System(Config config);
    ~System();
    System(const System&) = delete;
    System& operator=(const System&) = delete;
    void submit(const Intent& intent);
    Status propulsion_status() const;
    Status bending_status() const;
    void shutdown();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

namespace inspire {
struct State { int target = 0, position = 0, temperature = 0, fault = 0; };
std::vector<unsigned char> command(int id, int op, const std::vector<unsigned char>& payload = {});
std::vector<unsigned char> registers(int id, int address, const std::vector<int>& values);
// Removes echoes/garbage and returns only a complete, matching, valid state.
bool extract(std::vector<unsigned char>& buffer, int id, int op, int address, State& state);
}
} // namespace robot
