#include "robot.hpp"
#include <stdexcept>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

namespace robot {
namespace {
class Keyboard final : public InputSource {
    termios previous_{};
    int flags_ = 0;
    Input latest_;
    bool commands_;
    std::string line_;
    Clock::time_point last_key_ = Clock::now();
public:
    explicit Keyboard(bool commands) : commands_(commands) {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &previous_)) throw std::runtime_error("keyboard mode requires a terminal");
        termios raw = previous_; raw.c_lflag &= ~(ICANON | ECHO); raw.c_cc[VMIN] = 0; raw.c_cc[VTIME] = 0;
        flags_ = fcntl(STDIN_FILENO, F_GETFL);
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) || fcntl(STDIN_FILENO, F_SETFL, flags_ | O_NONBLOCK)) {
            tcsetattr(STDIN_FILENO, TCSANOW, &previous_); throw std::runtime_error("cannot configure terminal");
        }
    }
    ~Keyboard() override { tcsetattr(STDIN_FILENO, TCSANOW, &previous_); fcntl(STDIN_FILENO, F_SETFL, flags_); }
    Input poll() override {
        if (commands_) {
            Input command;
            char key;
            while (::read(STDIN_FILENO, &key, 1) == 1) {
                if (key == 27) { line_.clear(); command.stop = true; continue; }
                if (key == 127 || key == 8) {
                    if (!line_.empty()) { line_.pop_back(); std::cout << "\b \b" << std::flush; }
                    continue;
                }
                if (key != '\n' && key != '\r') {
                    if (line_.size() >= 256) { line_.clear(); command.stop = true; }
                    else { line_ += key; std::cout << key << std::flush; }
                    continue;
                }
                std::cout << '\n';
                std::istringstream stream(line_);
                std::string op, extra;
                stream >> op;
                if (op == "stop") command.stop = true;
                else if (op == "quit") command.quit = true;
                else if (op == "enable" && !(stream >> extra)) command.enable = true;
                else if (op == "move") {
                    std::array<double, 3> values;
                    if ((stream >> values[0] >> values[1] >> values[2]) && !(stream >> extra) &&
                        std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); }))
                        command.position = values;
                    else { command.stop = true; std::cout << "Rejected: use move X1 X2 X3 (mm); stopped.\n"; }
                } else if (!op.empty()) { command.stop = true; std::cout << "Unknown command; stopped.\n"; }
                line_.clear();
            }
            return command;
        }
        Input out;
        if (seconds(last_key_) < 0.15) out = latest_;
        out.enable = out.stop = out.quit = false;
        bool requested_stop = false, requested_quit = false;
        char key;
        while (::read(STDIN_FILENO, &key, 1) == 1) {
            out = Input{};
            switch (key) {
                case 'e': out.enable = true; break;
                case 'q': requested_quit = true; break;
                case 'x': case ' ': requested_stop = true; break;
                case 'd': out.propulsion = 1; break;
                case 'a': out.propulsion = -1; break;
                case '1': out.cylinders[0] = 1; break;
                case '2': out.cylinders[0] = -1; break;
                case '3': out.cylinders[1] = 1; break;
                case '4': out.cylinders[1] = -1; break;
                case '5': out.cylinders[2] = 1; break;
                case '6': out.cylinders[2] = -1; break;
                default: break;
            }
            out.held = out.propulsion != 0 || out.cylinders != std::array<double, 3>{};
            latest_ = out; last_key_ = Clock::now();
        }
        // A later direction key in the same terminal read cannot erase a stop.
        out.stop = requested_stop; out.quit = requested_quit;
        return out;
    }
    std::string description() const override { return commands_ ? "commands: enable; move X1 X2 X3 (mm); stop; quit; Enter submits; Esc stops immediately" : "keyboard: e enable; a/d propulsion; 1/2,3/4,5/6 cylinders; space/x stop; q quit"; }
};
}
std::unique_ptr<InputSource> keyboard(bool commands) { return std::make_unique<Keyboard>(commands); }
}
