#include "serial.hpp"
#include <algorithm>
#include <cerrno>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace robot {
namespace {
std::runtime_error error(const std::string& what) { return std::runtime_error(what + ": " + std::strerror(errno)); }
}
Serial::Serial(const std::string& path, int baud) {
    speed_t speed;
    switch (baud) {
        case 19200: speed = B19200; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
#ifdef B921600
        case 921600: speed = B921600; break;
#endif
        default: throw std::invalid_argument("unsupported serial baudrate on this platform");
    }
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) throw error("open " + path);
    try {
        // Exclusive ownership is released when this descriptor closes.
        if (ioctl(fd_, TIOCEXCL) < 0) throw error("exclusive serial access");
        termios t{};
        if (tcgetattr(fd_, &t)) throw error("read serial settings");
        cfmakeraw(&t); cfsetispeed(&t, speed); cfsetospeed(&t, speed);
        t.c_cflag = (t.c_cflag & ~(CSIZE | PARENB | CSTOPB | CRTSCTS)) | CS8 | CLOCAL | CREAD;
        t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &t)) throw error("set serial settings");
        tcflush(fd_, TCIOFLUSH);
    } catch (...) { ::close(fd_); fd_ = -1; throw; }
}
Serial::~Serial() { if (fd_ >= 0) { ioctl(fd_, TIOCNXCL); ::close(fd_); } }
void Serial::write(const std::vector<unsigned char>& data) {
    auto started = Clock::now();
    size_t n = 0;
    while (n < data.size()) {
        if (seconds(started) > 0.1) throw std::runtime_error("serial write timeout");
        ssize_t count = ::write(fd_, data.data() + n, data.size() - n);
        if (count > 0) { n += static_cast<size_t>(count); continue; }
        if (count < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) throw error("serial write");
        pollfd p{fd_, POLLOUT, 0}; poll(&p, 1, 5);
    }
}
std::vector<unsigned char> Serial::read(int timeout_ms) {
    pollfd p{fd_, POLLIN, 0};
    int ready = poll(&p, 1, timeout_ms);
    if (ready < 0 && errno != EINTR) throw error("serial poll");
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) throw std::runtime_error("serial disconnected");
    if (ready <= 0 || !(p.revents & POLLIN)) return {};
    unsigned char b[256];
    ssize_t n = ::read(fd_, b, sizeof(b));
    if (n < 0 && errno != EAGAIN && errno != EINTR) throw error("serial read");
    return n > 0 ? std::vector<unsigned char>(b, b + n) : std::vector<unsigned char>{};
}
std::string Serial::line(const std::string& cmd, const std::string& expected) {
    // Previous complete lines cannot acknowledge a new transaction.
    lines_.clear(); tcflush(fd_, TCIFLUSH);
    std::string wire = cmd + "\n";
    write(std::vector<unsigned char>(wire.begin(), wire.end()));
    auto start = Clock::now();
    while (seconds(start) < 0.2) {
        auto bytes = read(5); lines_.append(bytes.begin(), bytes.end());
        size_t end;
        while ((end = lines_.find('\n')) != std::string::npos) {
            std::string reply = lines_.substr(0, end); lines_.erase(0, end + 1);
            if (!reply.empty() && reply.back() == '\r') reply.pop_back();
            if (reply.rfind("ERR", 0) == 0) throw std::runtime_error(cmd + ": " + reply);
            if (expected == "V" && reply.rfind("V", 0) == 0) {
                size_t used = 0;
                long speed = std::stol(reply.substr(1), &used);
                if (used != reply.size() - 1 || speed < -16000 || speed > 16000)
                    throw std::runtime_error("invalid Nano speed reply");
                return reply;
            }
            if (reply == expected || (expected == "INFO PCB1_S3 " && reply.rfind(expected, 0) == 0)) return reply;
        }
        if (lines_.size() > 4096) throw std::runtime_error("oversized serial response");
    }
    throw std::runtime_error("no matching reply to " + cmd);
}
inspire::State Serial::exchange(int id, const std::vector<unsigned char>& request, int op, int address) {
    tcflush(fd_, TCIFLUSH);
    write(request);
    auto start = Clock::now();
    std::vector<unsigned char> buffer, sample;
    size_t received = 0;
    while (seconds(start) < 0.15) {
        auto chunk = read(5); buffer.insert(buffer.end(), chunk.begin(), chunk.end());
        received += chunk.size();
        for (auto byte : chunk) if (sample.size() < 64) sample.push_back(byte);
        inspire::State state;
        if (inspire::extract(buffer, id, op, address, state)) {
            if (state.fault) throw std::runtime_error("cylinder " + std::to_string(id) + " fault=" + std::to_string(state.fault));
            return state;
        }
    }
    std::ostringstream detail;
    detail << "cylinder " << id << " response timeout/invalid frame: op=0x" << std::hex << op
           << " address=0x" << address << std::dec << " received_bytes=" << received
           << " reason=" << (received == 0 ? "no_bytes" : received < 20 ? "short_reply" : "no_matching_valid_frame")
           << " rx_hex=";
    for (auto byte : sample) detail << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(byte);
    if (received > sample.size()) detail << "...";
    throw std::runtime_error(detail.str());
}
} // namespace robot
