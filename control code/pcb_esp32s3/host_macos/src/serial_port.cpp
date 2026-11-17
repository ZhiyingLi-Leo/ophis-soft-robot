#include "serial_port.hpp"

#include <cerrno>
#include <chrono>
#include <sys/ioctl.h>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace mac_control {
namespace {

bool baud_to_termios(int baud_rate, speed_t &speed) {
    switch (baud_rate) {
    case 9600: speed = B9600; return true;
    case 57600: speed = B57600; return true;
    case 115200: speed = B115200; return true;
    default: return false;
    }
}

} // namespace

SerialPort::~SerialPort() {
    close_port();
}

bool SerialPort::open_port(const std::string &path, int baud_rate, std::string &error) {
    close_port();
    speed_t speed{};
    if (!baud_to_termios(baud_rate, speed)) {
        error = "不支持的波特率: " + std::to_string(baud_rate);
        return false;
    }

    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        error = "无法打开串口 " + path + ": " + std::strerror(errno);
        return false;
    }

    termios options{};
    if (tcgetattr(fd_, &options) != 0) {
        error = "tcgetattr 失败: " + std::string(std::strerror(errno));
        close_port();
        return false;
    }
    cfmakeraw(&options);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
        error = "tcsetattr 失败: " + std::string(std::strerror(errno));
        close_port();
        return false;
    }
    int dtr = TIOCM_DTR;
    ioctl(fd_, TIOCMBIS, &dtr);
    tcflush(fd_, TCIOFLUSH);
    return true;
}

bool SerialPort::send_line(const std::string &line, std::string &error) {
    if (fd_ < 0) {
        error = "串口未打开";
        return false;
    }
    const std::string payload = line + "\n";
    std::size_t written = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (written < payload.size()) {
        if (std::chrono::steady_clock::now() >= deadline) { error = "串口写入超时"; return false; }
        const ssize_t result = ::write(fd_, payload.data() + written, payload.size() - written);
        if (result > 0) {
            written += static_cast<std::size_t>(result);
            continue;
        }
        if (result < 0 && (errno == EINTR || errno == EAGAIN)) {
            usleep(1000);
            continue;
        }
        error = "串口写入失败: " + std::string(std::strerror(errno));
        return false;
    }
    return true;
}

std::string SerialPort::read_available(std::string &error) {
    std::string received;
    if (fd_ < 0) return received;

    char buffer[128];
    while (true) {
        const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
        if (count > 0) {
            received.append(buffer, static_cast<std::size_t>(count));
            continue;
        }
        if (count == 0 || (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
            break;
        }
        if (count < 0 && errno == EINTR) continue;
        error = "串口读取失败: " + std::string(std::strerror(errno));
        break;
    }
    return received;
}

void SerialPort::close_port() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

} // namespace mac_control
