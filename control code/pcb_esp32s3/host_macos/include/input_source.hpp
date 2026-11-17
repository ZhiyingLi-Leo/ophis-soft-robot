#pragma once

#include <memory>
#include <string>

namespace mac_control {

struct InputSample {
    double propulsion_axis = 0.0;
    bool emergency_stop = false;
    bool quit = false;
};

class InputSource {
public:
    virtual ~InputSource() = default;
    virtual InputSample poll() = 0;
    virtual std::string description() const = 0;
    virtual bool connected() const = 0;
};

std::unique_ptr<InputSource> make_simulated_input(bool automatic_demo);
std::unique_ptr<InputSource> make_macos_gamepad(int axis_usage, int stop_button_usage);

} // namespace mac_control
