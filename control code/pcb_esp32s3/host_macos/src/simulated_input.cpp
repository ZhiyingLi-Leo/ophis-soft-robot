#include "input_source.hpp"

#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

namespace mac_control {
namespace {

class TerminalRawMode {
public:
    TerminalRawMode() {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &original_) != 0) {
            return;
        }
        termios raw = original_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
            original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
            fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK);
            active_ = true;
        }
    }

    ~TerminalRawMode() {
        if (active_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_);
            fcntl(STDIN_FILENO, F_SETFL, original_flags_);
        }
    }

private:
    termios original_{};
    int original_flags_ = 0;
    bool active_ = false;
};

class SimulatedInput final : public InputSource {
public:
    explicit SimulatedInput(bool automatic_demo)
        : automatic_demo_(automatic_demo), start_(std::chrono::steady_clock::now()) {
        if (!automatic_demo_) {
            std::cout << "仿真手柄: [a]反向  [d]正向  [s/空格]回中  [x]急停  [q]退出\n";
            std::cout << "每次 a/d 改变 10% 摇杆量。\n";
        }
    }

    InputSample poll() override {
        if (automatic_demo_) {
            return poll_demo();
        }

        InputSample sample;
        char key = 0;
        while (::read(STDIN_FILENO, &key, 1) == 1) {
            switch (key) {
            case 'a': axis_ = std::max(-1.0, axis_ - 0.1); break;
            case 'd': axis_ = std::min(1.0, axis_ + 0.1); break;
            case 's':
            case ' ': axis_ = 0.0; break;
            case 'x': axis_ = 0.0; sample.emergency_stop = true; break;
            case 'q': sample.quit = true; break;
            default: break;
            }
        }
        sample.propulsion_axis = axis_;
        return sample;
    }

    std::string description() const override {
        return automatic_demo_ ? "自动仿真手柄" : "键盘仿真手柄";
    }

    bool connected() const override { return true; }

private:
    InputSample poll_demo() {
        const double seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_).count();
        InputSample sample;
        if (seconds < 0.5) sample.propulsion_axis = 0.0;
        else if (seconds < 1.5) sample.propulsion_axis = 0.6;
        else if (seconds < 2.0) sample.propulsion_axis = 0.0;
        else if (seconds < 3.0) sample.propulsion_axis = -0.4;
        else if (seconds < 3.5) sample.propulsion_axis = 0.0;
        else sample.quit = true;
        return sample;
    }

    TerminalRawMode terminal_;
    bool automatic_demo_ = false;
    double axis_ = 0.0;
    std::chrono::steady_clock::time_point start_;
};

} // namespace

std::unique_ptr<InputSource> make_simulated_input(bool automatic_demo) {
    return std::make_unique<SimulatedInput>(automatic_demo);
}

} // namespace mac_control
