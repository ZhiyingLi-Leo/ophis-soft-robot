#include "control_logic.hpp"
#include "input_source.hpp"
#include "serial_port.hpp"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace {

volatile std::sig_atomic_t g_interrupted = 0;

void signal_handler(int) {
    g_interrupted = 1;
}

struct Options {
    std::string mode = "simulate";
    std::string serial_path;
    int baud_rate = 115200;
    int max_speed = 3200;
    double deadzone = 0.08;
    double acceleration = 12000.0;
    int axis_usage = 0x33;       // Rx
    int stop_button_usage = 7;
    bool demo = false;
    bool diagnostic = false;
    double duration_seconds = 0.0;
};

void print_usage(const char *program) {
    std::cout
        << "用法: " << program << " [选项]\n"
        << "  --mode simulate|gamepad  键盘仿真或 macOS GameController 真手柄\n"
        << "  --serial /dev/cu.xxx    PCB 串口；不填即 dry-run\n"
        << "  --max-speed N           最高速度，默认 3200 步/秒\n"
        << "  --acceleration N        最大加速度，默认 12000 步/秒^2\n"
        << "  --deadzone X            摇杆死区，默认 0.08\n"
        << "  --axis-usage N          摇杆映射，默认 0x33（右摇杆水平）\n"
        << "  --stop-button N         急停按键映射，默认 7（Options/Back）\n"
        << "  --demo                   运行自动仿真并退出\n"
        << "  --diagnostic             显示手柄连接、原始轴值和速度\n"
        << "  --duration X             X 秒后自动退出，0 表示持续运行\n";
}

long parse_integer(const std::string &text) {
    std::size_t consumed = 0;
    const long value = std::stol(text, &consumed, 0);
    if (consumed != text.size()) throw std::invalid_argument("invalid integer");
    return value;
}

const char *travel_state(int speed) {
    if (speed > 0) return "导管前进";
    if (speed < 0) return "导管回退";
    return "导管停止";
}

const char *upper_direction_level(int speed) {
    if (speed > 0) return "GPIO5/14=HIGH";
    if (speed < 0) return "GPIO5/14=LOW";
    return "上组保持";
}

const char *lower_direction_level(int speed) {
    if (speed > 0) return "GPIO10/36=LOW";
    if (speed < 0) return "GPIO10/36=HIGH";
    return "下组保持";
}

Options parse_options(int argc, char **argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (++i >= argc) throw std::invalid_argument("选项缺少参数: " + arg);
            return argv[i];
        };
        if (arg == "--mode") options.mode = next();
        else if (arg == "--serial") options.serial_path = next();
        else if (arg == "--max-speed") options.max_speed = static_cast<int>(parse_integer(next()));
        else if (arg == "--acceleration") options.acceleration = std::stod(next());
        else if (arg == "--deadzone") options.deadzone = std::stod(next());
        else if (arg == "--axis-usage") options.axis_usage = static_cast<int>(parse_integer(next()));
        else if (arg == "--stop-button") options.stop_button_usage = static_cast<int>(parse_integer(next()));
        else if (arg == "--demo") options.demo = true;
        else if (arg == "--diagnostic") options.diagnostic = true;
        else if (arg == "--duration") options.duration_seconds = std::stod(next());
        else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            throw std::invalid_argument("未知选项: " + arg);
        }
    }
    if (options.mode != "simulate" && options.mode != "gamepad") {
        throw std::invalid_argument("--mode 只能是 simulate 或 gamepad");
    }
    if (options.max_speed <= 0 || options.acceleration <= 0.0 ||
        options.deadzone < 0.0 || options.deadzone >= 1.0 || options.duration_seconds < 0.0) {
        throw std::invalid_argument("速度、加速度或死区参数无效");
    }
    return options;
}

} // namespace

int main(int argc, char **argv) {
    Options options;
    try {
        options = parse_options(argc, argv);
    } catch (const std::exception &error) {
        std::cerr << "参数错误: " << error.what() << "\n";
        print_usage(argv[0]);
        return 2;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    mac_control::ControlConfig config;
    config.max_speed_steps_per_second = options.max_speed;
    config.deadzone = options.deadzone;
    config.max_acceleration_steps_per_second_squared = options.acceleration;

    std::unique_ptr<mac_control::InputSource> input;
    if (options.mode == "gamepad") {
        input = mac_control::make_macos_gamepad(options.axis_usage, options.stop_button_usage);
    } else {
        input = mac_control::make_simulated_input(options.demo);
    }

    mac_control::SerialPort serial;
    if (!options.serial_path.empty()) {
        std::string error;
        if (!serial.open_port(options.serial_path, options.baud_rate, error)) {
            std::cerr << error << "\n";
            return 1;
        }
    }

    // Only the PCB firmware may be armed; the Nano remains on its original program.
    if (serial.is_open()) {
        auto exchange = [&](const std::string &command, const std::string &expected) {
            std::string error, reply;
            if (!serial.send_line(command, error)) return false;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (std::chrono::steady_clock::now() < deadline) {
                reply += serial.read_available(error);
                if (!error.empty() || reply.find("ERR") != std::string::npos) return false;
                if (reply.find(expected) != std::string::npos) return true;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            return false;
        };
        // Detect the board now. Arm only after input detection below.
        if (!exchange("INFO", "INFO PCB1_S3 ")) {
            std::cerr << "新PCB固件身份检查失败；未解锁电机。\n";
            return 1;
        }
    }

    // GameController 的设备列表异步更新。启动时给 macOS 最多 2 秒完成识别，
    // 避免手柄已经配对却因第一次同步查询为空而误报“未连接”。
    if (options.mode == "gamepad") {
        const auto detection_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!input->connected() && std::chrono::steady_clock::now() < detection_deadline) {
            (void)input->poll();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    const mac_control::InputSample initial_sample = input->poll();
    (void)initial_sample;
    std::cout << "输入源: " << input->description() << "\n"
              << "PCB: " << (serial.is_open() ? options.serial_path : "dry-run（不连接真机）") << "\n";
    if (options.mode == "gamepad" && !input->connected()) {
        std::cout << "警告: 当前还不能读取手柄；若上方提示权限，请在 macOS 的"
                     "“隐私与安全性 → 输入监控”中授权运行本程序的终端或 VS Code。\n";
    }

    if (serial.is_open()) {
        if (options.mode == "gamepad" && !input->connected()) {
            std::cerr << "未检测到手柄，拒绝解锁。\n"; return 1;
        }
        std::string error, reply;
        if (!serial.send_line("A1", error)) return 1;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline && reply.find("OK A1") == std::string::npos) {
            reply += serial.read_available(error);
            if (!error.empty() || reply.find("ERR") != std::string::npos) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (reply.find("OK A1") == std::string::npos) {
            serial.send_line("S", error); serial.send_line("A0", error);
            std::cerr << "解锁失败：" << reply << "\n"; return 1;
        }
    }

    int commanded_speed = 0;
    int last_sent_speed = -1;
    auto previous = std::chrono::steady_clock::now();
    const auto program_start = previous;
    auto last_diagnostic = previous - std::chrono::seconds(1);
    bool running = true;
    auto last_heartbeat = previous;
    std::string response_buffer;

    auto send_speed = [&](int speed, bool force) -> bool {
        if (!force && speed == last_sent_speed) return true;
        const std::string command = "V" + std::to_string(speed);
        if (serial.is_open()) {
            std::string error;
            if (!serial.send_line(command, error)) {
                std::cerr << error << "\n";
                return false;
            }
        }
        std::cout << "\r摇杆控制 -> " << command << " 步/秒 | "
                  << travel_state(speed) << "      " << std::flush;
        last_sent_speed = speed;
        return true;
    };

    // 连接后首先显式发送停止指令，不依赖 PCB 的上电状态。
    if (!send_speed(0, true)) return 1;

    while (running && !g_interrupted) {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - previous).count();
        previous = now;

        const mac_control::InputSample sample = input->poll();
        const int target_speed = mac_control::axis_to_speed(sample.propulsion_axis, config);
        if (sample.emergency_stop || !input->connected()) {
            commanded_speed = 0;
            if (!send_speed(0, true)) break;
        } else {
            commanded_speed = mac_control::limit_speed_change(commanded_speed, target_speed, dt, config);
            if (!send_speed(commanded_speed, false)) break;
        }
        if (sample.quit) running = false;
        if (options.duration_seconds > 0.0 &&
            std::chrono::duration<double>(now - program_start).count() >= options.duration_seconds) {
            running = false;
        }

        if (options.diagnostic && now - last_diagnostic >= std::chrono::milliseconds(250)) {
            std::cout << "\n诊断: connected=" << (input->connected() ? "yes" : "no")
                      << " axis=" << sample.propulsion_axis
                      << " target=" << target_speed
                      << " command=" << commanded_speed
                      << " travel=" << travel_state(commanded_speed)
                      << " upper=" << upper_direction_level(commanded_speed)
                      << " lower=" << lower_direction_level(commanded_speed)
                      << " emergency=" << (sample.emergency_stop ? "yes" : "no") << std::flush;
            last_diagnostic = now;
        }

        if (serial.is_open() && now - last_heartbeat >= std::chrono::milliseconds(200)) {
            std::string heartbeat_error;
            if (!serial.send_line("H", heartbeat_error)) break;
            last_heartbeat = now;
        }
        // Drain acknowledgements and fail closed on a rejected command.
        if (serial.is_open()) {
            std::string read_error;
            const std::string reply = serial.read_available(read_error);
            if (!read_error.empty()) {
                std::cerr << "\n" << read_error << "\n";
                break;
            }
            response_buffer += reply;
            bool rejected = false;
            std::size_t newline = 0;
            while ((newline = response_buffer.find('\n')) != std::string::npos) {
                const auto line = response_buffer.substr(0, newline);
                response_buffer.erase(0, newline + 1);
                if (line.rfind("ERR", 0) == 0) { std::cerr << "\nPCB 返回: " << line; rejected = true; }
            }
            if (rejected || response_buffer.size() > 4096) {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // 无论正常退出还是 Ctrl-C，都在关串口前发送 V0。
    send_speed(0, true);
    if (serial.is_open()) { std::string error; serial.send_line("A0", error); }
    std::cout << "\n控制程序已停止。\n";
    return 0;
}
