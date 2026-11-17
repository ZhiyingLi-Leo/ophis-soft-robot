#include "robot.hpp"
#include "serial.hpp"
#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void signal_handler(int) { interrupted = 1; }
struct Options { std::string mode = "keyboard", config, record, propulsion; double duration = 0; };
double number(const std::string& s) {
    size_t end; double x = std::stod(s, &end);
    if (end != s.size() || !std::isfinite(x) || x < 0) throw std::invalid_argument("invalid duration");
    return x;
}
Options options(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];
        if (key == "--help") {
            std::cout << "robot_control [--mode keyboard|command|inspect|gamepad|demo] [--propulsion nano|pcb|none]\n"
                         "              [--config hardware.conf] [--duration seconds] [--record new.csv]\n"
                         "propulsion=none: cylinders only, no propulsion device opened.\nNo --config: simulated devices only. --demo never opens hardware.\n";
            std::exit(0);
        }
        if (++i >= argc) throw std::invalid_argument("missing value for " + key);
        std::string value = argv[i];
        if (key == "--mode") o.mode = value;
        else if (key == "--config") o.config = value;
        else if (key == "--record") o.record = value;
        else if (key == "--propulsion") o.propulsion = value;
        else if (key == "--duration") o.duration = number(value);
        else throw std::invalid_argument("unknown option " + key);
    }
    if (o.mode != "keyboard" && o.mode != "command" && o.mode != "inspect" && o.mode != "gamepad" && o.mode != "demo") throw std::invalid_argument("invalid input mode");
    if (o.mode == "demo" && !o.config.empty()) throw std::invalid_argument("demo does not accept hardware configuration");
    if (o.mode == "inspect" && o.config.empty()) throw std::invalid_argument("inspect requires --config");
    return o;
}
struct Recording {
    FILE* file = nullptr;
    explicit Recording(const std::string& path) {
        if (path.empty()) return;
        int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
        if (fd < 0) throw std::runtime_error("cannot create new recording: " + path);
        file = fdopen(fd, "w");
        if (!file) { close(fd); throw std::runtime_error("cannot open recording stream"); }
        if (std::fputs("elapsed_s,simulation,active,propulsion_command_sps,cylinder1_mm,cylinder2_mm,cylinder3_mm,target1_mm,target2_mm,target3_mm,propulsion_ready,bending_ready,cylinder1_age_s,cylinder2_age_s,cylinder3_age_s,homing\n", file) < 0) {
            std::fclose(file); file = nullptr; throw std::runtime_error("cannot write recording header");
        }
    }
    ~Recording() { if (file) std::fclose(file); }
    void row(double t, bool simulation, bool active, const robot::Status& p, const robot::Status& b, bool homing = false) {
        if (!file) return;
        if (std::fprintf(file, "%.6f,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.6f,%.6f,%.6f,%d\n", t, simulation, active, p.speed,
            b.position[0], b.position[1], b.position[2], b.target[0], b.target[1], b.target[2], p.ready, b.ready,
            b.feedback_time[0] == robot::Clock::time_point{} ? -1 : robot::seconds(b.feedback_time[0]),
            b.feedback_time[1] == robot::Clock::time_point{} ? -1 : robot::seconds(b.feedback_time[1]),
            b.feedback_time[2] == robot::Clock::time_point{} ? -1 : robot::seconds(b.feedback_time[2]), homing) < 0 || std::fflush(file))
            throw std::runtime_error("recording write failed");
    }
};
}
int main(int argc, char** argv) {
    try {
        auto o = options(argc, argv);
        auto config = robot::load_config(o.config);
        if (!o.propulsion.empty()) {
            if (config.hardware && o.propulsion != config.propulsion) throw std::invalid_argument("hardware backend must match configuration");
            config.propulsion = o.propulsion;
        }
        if (o.mode == "command" && config.propulsion != "none")
            throw std::invalid_argument("command mode requires propulsion=none");
        robot::validate(config);
        if (config.hardware && config.cylinder_port == "auto") {
            config.cylinder_port = robot::discover_cylinder_port(config, robot::cylinder_port_candidates());
            std::cout << "Detected cylinder port: " << config.cylinder_port << std::endl;
            robot::validate(config);
        }
        if (o.mode == "inspect") {
            std::cout << "READ ONLY: status queries only; auto_zero ignored; no motion or register writes.\n";
            robot::Serial port(config.cylinder_port, config.cylinder_baud);
            bool invalid = false;
            for (const auto& c : config.cylinders) {
                try {
                    const auto state = port.exchange(c.id, robot::inspire::command(c.id, 0x30));
                    const double mm = state.position * c.stroke / 2000;
                    const bool within = mm >= c.minimum && mm <= c.maximum;
                    const bool accepted = robot::valid_feedback(mm, c, config.zero_tolerance_mm);
                    std::cout << "ID=" << c.id << " raw_position=" << state.position
                              << " raw_target=" << state.target << " calculated_mm=" << mm
                              << " limits_mm=[" << c.minimum << ',' << c.maximum << "]"
                              << " within_limits=" << within << " feedback_accepted=" << accepted
                              << " zero_tolerance_mm=" << config.zero_tolerance_mm << " temperature=" << state.temperature << '\n';
                    invalid = invalid || !accepted;
                } catch (const std::exception& e) {
                    invalid = true; std::cout << "ID=" << c.id << " ERROR: " << e.what() << '\n';
                }
            }
            return invalid ? 1 : 0;
        }
        if (config.hardware) {
            char p[4096], c[4096];
            if (!realpath(config.cylinder_port.c_str(), c))
                throw std::runtime_error("configured serial device does not exist");
            if (config.propulsion != "none") {
                if (!realpath(config.propulsion_port.c_str(), p))
                    throw std::runtime_error("configured propulsion device does not exist");
                if (std::string(p) == c) throw std::invalid_argument("serial ports refer to the same device");
            }
        }
        Recording recording(o.record);
        std::signal(SIGINT, signal_handler); std::signal(SIGTERM, signal_handler);
        std::unique_ptr<robot::InputSource> source;
        if (o.mode == "gamepad") source = robot::gamepad();
        if (o.mode == "keyboard" || o.mode == "command") source = robot::keyboard(o.mode == "command");
        robot::System system(config);
        std::cout << (config.hardware ? "HARDWARE" : "SIMULATION") << " propulsion=" << config.propulsion << " (step/s); bending=three cylinder positions (mm)\n";
        if (source) std::cout << source->description() << '\n';
        if (o.mode == "gamepad") std::cout << "Gamepad: A enable at neutral, hold L1 to move; Options stop; Ctrl-C quit.\n" << std::flush;
        bool enabled = false, previous_enable = false, failed = false;
        std::optional<std::array<double, 3>> goal;
        bool arrived = false;
        double move_start = 0;
        bool homing = config.auto_zero, homing_started = false;
        double homing_start = 0, homing_timeout = 10;
        for (const auto& c : config.cylinders)
            homing_timeout = std::max(homing_timeout, 10 + 4 * c.stroke / c.speed);
        if (homing) std::cout << "Auto-zero: moving to absolute 0 mm after readiness; "
            << (o.mode == "command" ? "stop + Enter or Esc cancels.\n" : "space/x or Options cancels.\n");
        auto start = robot::Clock::now(), display = start - std::chrono::seconds(1);
        std::string previous_description;
        double duration = o.duration > 0 ? o.duration : (o.mode == "demo" ? 3.5 : 0);
        while (!interrupted) {
            double t = robot::seconds(start);
            if (duration > 0 && t >= duration) break;
            robot::Input input;
            if (source) input = source->poll();
            else {
                input.enable = t > 0.1 && t < 0.2;
                input.held = t > 0.4 && t < 2.8;
                input.propulsion = input.held ? (t < 1.5 ? 0.5 : -0.5) : 0;
                input.cylinders = input.held ? std::array<double, 3>{0.6, -0.4, 0.3} : std::array<double, 3>{};
                input.stop = t >= 2.8;
            }
            if (config.propulsion == "none") input.propulsion = 0;
            auto p = system.propulsion_status(), b = system.bending_status();
            if (!p.error.empty() || !b.error.empty()) {
                std::cerr << "\nSTOP: propulsion=" << p.error << " bending=" << b.error << '\n'; failed = true; break;
            }
            bool ready = b.ready && robot::seconds(b.time) < 1 &&
                (config.propulsion == "none" || (p.ready && robot::seconds(p.time) < 1));
            bool neutral = std::abs(input.propulsion) <= config.deadzone;
            for (auto v : input.cylinders) neutral = neutral && std::isfinite(v) && std::abs(v) <= config.deadzone;
            if (!ready || !input.connected || input.stop || input.quit) enabled = false;
            else if (input.enable && !previous_enable && neutral && !input.held) enabled = true;
            previous_enable = input.enable;
            bool active = enabled && input.held;
            if (homing) {
                enabled = false;
                active = false;
                input.propulsion = 0;
                input.cylinders = {};
                if (input.stop || input.quit || !input.connected || (homing_started && !ready)) {
                    homing = false;
                    std::cout << "\nAuto-zero cancelled; manual enable required.\n";
                } else if (ready) {
                    if (!homing_started) { homing_started = true; homing_start = t; }
                    const bool at_zero = std::all_of(b.position.begin(), b.position.end(),
                                                    [&](double mm) { return std::abs(mm) <= config.zero_tolerance_mm; });
                    if (at_zero) {
                        homing = false;
                        std::cout << "\nAuto-zero complete: all three positions are within configured zero tolerance; manual enable required.\n";
                    } else if (t - homing_start > homing_timeout) {
                        std::cerr << "\nSTOP: auto-zero timeout\n"; failed = true; break;
                    } else {
                        active = true;
                        for (int i = 0; i < 3; ++i) input.cylinders[i] = b.position[i] > config.zero_tolerance_mm ? -1 : 0;
                    }
                }
            }
            if (o.mode == "command") {
                if (!enabled) goal.reset();
                if (input.position) {
                    if (homing || !enabled) std::cout << "Rejected: wait for homing, then enable before move.\n";
                    else {
                        try {
                            for (int i = 0; i < 3; ++i) robot::position_code((*input.position)[i], config.cylinders[i]);
                            goal = input.position; arrived = false; move_start = t;
                            std::cout << "Target mm: " << (*goal)[0] << ' ' << (*goal)[1] << ' ' << (*goal)[2] << '\n';
                        } catch (const std::exception& e) {
                            goal.reset(); enabled = false;
                            std::cout << "Rejected: " << e.what() << "; stopped, enable required.\n";
                        }
                    }
                }
                if (goal) {
                    active = true;
                    bool reached = true;
                    for (int i = 0; i < 3; ++i) {
                        const auto& c = config.cylinders[i];
                        const double encoded = robot::position_code((*goal)[i], c) * c.stroke / 2000;
                        reached = reached && std::abs(b.position[i] - encoded) <= (encoded == 0 ? std::max(c.stroke / 4000, config.zero_tolerance_mm) : c.stroke / 4000);
                    }
                    if (reached && !arrived) {
                        arrived = true;
                        std::cout << "Reached target (encoder feedback); holding position command.\n";
                    }
                    if (!arrived && t - move_start > homing_timeout) {
                        std::cerr << "\nSTOP: position move timeout\n"; failed = true; break;
                    }
                }
            }
            robot::Intent intent{active, input.propulsion, input.cylinders, robot::Clock::now(), goal};
            system.submit(intent);
            recording.row(t, !config.hardware, active, p, b, homing);
            if (source && source->description() != previous_description) {
                previous_description = source->description(); std::cout << '\n' << previous_description << '\n';
            }
            if (robot::seconds(display) >= (o.mode == "command" ? 1.0 : 0.25)) {
                std::cout << std::fixed << std::setprecision(3) << "\rt=" << t << " ready=" << ready << " enabled=" << enabled
                          << " active=" << active << " homing=" << homing << " propulsion=" << p.speed << " position_mm=[" << b.position[0] << ','
                          << b.position[1] << ',' << b.position[2] << "]       " << std::flush;
                display = robot::Clock::now();
            }
            if (input.quit) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        system.submit({}); system.shutdown();
        auto p = system.propulsion_status(), b = system.bending_status();
        if (!p.error.empty() || !b.error.empty()) {
            std::cerr << "\nShutdown: propulsion=" << p.error << " bending=" << b.error << '\n'; failed = true;
        }
        recording.row(robot::seconds(start), !config.hardware, false, p, b);
        std::cout << "\nController closed; verify mechanical stop on hardware.\n";
        return failed ? 1 : 0;
    } catch (const std::exception& e) { std::cerr << "ERROR: " << e.what() << '\n'; return 1; }
}
