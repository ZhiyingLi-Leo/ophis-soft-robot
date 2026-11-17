#include "robot.hpp"
#include "serial.hpp"
#include <algorithm>
#include <atomic>
#include <mutex>
#include <sstream>
#include <thread>

namespace robot {
struct System::Impl {
    Config config;
    mutable std::mutex mutex;
    Intent intent;
    Status propulsion, bending;
    std::atomic<bool> running{true};
    std::atomic<bool> p_stop{false}, b_stop{false};
    std::thread p_thread, b_thread;

    explicit Impl(Config c) : config(std::move(c)) {
        validate(config);
        if (config.propulsion != "none") p_thread = std::thread([this] { propulsion_loop(); });
        try { b_thread = std::thread([this] { bending_loop(); }); }
        catch (...) { running = false; if (p_thread.joinable()) p_thread.join(); throw; }
    }
    Intent request() const {
        std::lock_guard<std::mutex> lock(mutex);
        auto copy = intent;
        if (!running || seconds(copy.time) > 0.25) copy.active = false;
        return copy;
    }
    void publish(bool p, Status status) {
        status.time = Clock::now();
        std::lock_guard<std::mutex> lock(mutex);
        (p ? propulsion : bending) = std::move(status);
    }
    void pause(double duration) const {
        auto start = Clock::now();
        while (running && seconds(start) < duration) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    void propulsion_loop() {
        Status status;
        std::unique_ptr<Serial> port;
        bool identified = false, armed = false;
        auto cleanup = [&] {
            if (!port || !identified) return;
            std::string errors;
            try { port->line("V0", "OK V0"); } catch (const std::exception& e) { errors = e.what(); }
            if (config.propulsion == "pcb") {
                try { port->line("A0", "OK A0"); }
                catch (const std::exception& e) { errors += std::string("; ") + e.what(); }
            }
            if (!errors.empty()) status.error += " stop unconfirmed: " + errors;
            status.speed = 0; status.ready = false;
        };
        try {
            if (config.hardware) {
                port = std::make_unique<Serial>(config.propulsion_port, 115200);
                if (config.propulsion == "nano") {
                    pause(2); // Nano resets on serial open; do not send motion.
                    if (!running) { publish(true, status); return; }
                    port->line("?", "V");
                } else port->line("INFO", "INFO PCB1_S3 ");
                identified = true;
                port->line("V0", "OK V0");
            }
            status.ready = true;
            int last_speed = 0;
            auto previous = Clock::now(), last_send = previous;
            while (running) {
                auto in = request();
                if (p_stop.exchange(false)) in.active = false;
                double dt = std::chrono::duration<double>(Clock::now() - previous).count();
                previous = Clock::now();
                if (!in.active) {
                    status.speed = 0;
                    if (port && (last_speed || armed)) {
                        port->line("V0", "OK V0");
                        if (config.propulsion == "pcb") port->line("A0", "OK A0");
                    }
                    armed = false;
                } else {
                    if (port && config.propulsion == "pcb" && !armed) port->line("A1", "OK A1");
                    armed = true;
                    int target = static_cast<int>(std::lround(axis_value(in.propulsion, config.deadzone) * config.max_speed));
                    status.speed = ramp(status.speed, target, dt, config.acceleration);
                    if (port && (status.speed != last_speed || seconds(last_send) >= 0.1)) {
                        // Periodic acknowledged V commands also maintain the PCB watchdog.
                        std::string v = "V" + std::to_string(status.speed);
                        port->line(v, "OK " + v); last_send = Clock::now();
                    }
                }
                last_speed = status.speed;
                publish(true, status);
                pause(0.02);
            }
        } catch (const std::exception& e) { status.error = e.what(); status.ready = false; }
        cleanup(); publish(true, status);
    }
    void bending_loop() {
        Status status;
        std::unique_ptr<Serial> port;
        std::array<inspire::State, 3> states{};
        std::array<bool, 3> owned{};
        bool was_active = false;
        auto stop = [&](int i) {
            if (port) {
                auto id = config.cylinders[i].id;
                states[i] = port->exchange(id, inspire::registers(id, 0x1a, {1}), 0x32, 0x1a);
            }
            owned[i] = false;
        };
        auto cleanup = [&] {
            // Each axis is attempted independently; one timeout must not skip the others.
            for (int i = 0; i < 3; ++i) {
                try { if (owned[i]) stop(i); }
                catch (const std::exception& e) { status.error += " stop unconfirmed: " + std::string(e.what()); }
            }
            status.ready = false;
        };
        try {
            if (config.hardware) port = std::make_unique<Serial>(config.cylinder_port, config.cylinder_baud);
            for (int i = 0; i < 3; ++i) {
                const auto& c = config.cylinders[i];
                if (port) states[i] = port->exchange(c.id, inspire::command(c.id, 0x30));
                else states[i].position = position_code((c.minimum + c.maximum) / 2, c);
                status.position[i] = states[i].position * c.stroke / 2000;
                status.feedback_time[i] = Clock::now();
                if (!valid_feedback(status.position[i], c, config.zero_tolerance_mm))
                    throw std::runtime_error("cylinder " + std::to_string(c.id) + " actual position outside configured limits: raw=" + std::to_string(states[i].position) +
                        " calculated_mm=" + std::to_string(status.position[i]) + " allowed=[" +
                        std::to_string(c.minimum) + "," + std::to_string(c.maximum) + "]");
                status.target[i] = std::clamp(status.position[i], c.minimum, c.maximum);
            }
            status.ready = true;
            auto previous = Clock::now();
            while (running) {
                auto in = request();
                // A stop cannot disappear if input is re-enabled while a reply is pending.
                const bool forced_stop = b_stop.exchange(false);
                if (forced_stop) in.active = false;
                double dt = std::min(0.05, std::chrono::duration<double>(Clock::now() - previous).count());
                previous = Clock::now();
                if (was_active && !in.active) {
                    for (int i = 0; i < 3; ++i) if (owned[i]) stop(i);
                }
                for (int i = 0; i < 3; ++i) {
                    const auto& c = config.cylinders[i];
                    if (port) states[i] = port->exchange(c.id, inspire::command(c.id, 0x30));
                    status.position[i] = states[i].position * c.stroke / 2000;
                    status.feedback_time[i] = Clock::now();
                    if (!valid_feedback(status.position[i], c, config.zero_tolerance_mm))
                        throw std::runtime_error("cylinder " + std::to_string(c.id) + " exceeded configured limits");
                    // Recheck between transactions: stale intent or a stop replaces motion.
                    auto latest = request();
                    if (forced_stop) latest.active = false;
                    if (!latest.active) {
                        if (owned[i]) stop(i);
                        status.target[i] = std::clamp(status.position[i], c.minimum, c.maximum);
                        continue;
                    }
                    if (!was_active) status.target[i] = std::clamp(status.position[i], c.minimum, c.maximum);
                    double direction = axis_value(latest.cylinders[i], config.deadzone);
                    if (direction != 0 || latest.position) {
                        // Bound accumulated targets relative to measured motion, not just time.
                        double lead = c.speed * 0.2 + c.stroke / 2000;
                        double next = status.target[i] + direction * c.speed * dt;
                        if (latest.position) {
                            const double goal = position_code((*latest.position)[i], c) * c.stroke / 2000;
                            next = status.target[i] + std::clamp(goal - status.target[i], -c.speed * dt, c.speed * dt);
                        }
                        status.target[i] = std::clamp(next,
                                                    std::max(c.minimum, std::clamp(status.position[i], c.minimum, c.maximum) - lead),
                                                    std::min(c.maximum, std::clamp(status.position[i], c.minimum, c.maximum) + lead));
                    }
                    int target = position_code(status.target[i], c);
                    if (target != states[i].target && (direction != 0 || latest.position || owned[i])) {
                        int speed = static_cast<int>(std::floor(c.speed / c.stroke * 2000));
                        owned[i] = true; // Write may arrive even if its reply is lost.
                        if (port) {
                            auto reply = port->exchange(c.id, inspire::registers(c.id, 0x25, {2, 0, 0, speed, target}), 0x32, 0x25);
                            if (reply.target != target) throw std::runtime_error("cylinder rejected target");
                            states[i] = reply;
                        } else { states[i].target = target; states[i].position = target; }
                    }
                }
                was_active = !forced_stop && request().active;
                publish(false, status);
                // Reduce idle bus traffic; active control keeps its existing cadence.
                pause(was_active ? 0.02 : 0.1);
            }
        } catch (const std::exception& e) { status.error = e.what(); status.ready = false; }
        cleanup(); publish(false, status);
    }
    void shutdown() {
        running = false;
        if (p_thread.joinable()) p_thread.join();
        if (b_thread.joinable()) b_thread.join();
    }
    ~Impl() { shutdown(); }
};
System::System(Config config) : impl_(std::make_unique<Impl>(std::move(config))) {}
System::~System() = default;
void System::submit(const Intent& in) {
    if (!std::isfinite(in.propulsion)) throw std::invalid_argument("invalid propulsion intent");
    for (double v : in.cylinders) if (!std::isfinite(v)) throw std::invalid_argument("invalid bending intent");
    if (in.position) for (int i = 0; i < 3; ++i) position_code((*in.position)[i], impl_->config.cylinders[i]);
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->intent.active && !in.active) { impl_->p_stop = true; impl_->b_stop = true; }
    impl_->intent = in;
}
Status System::propulsion_status() const { std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->propulsion; }
Status System::bending_status() const { std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->bending; }
void System::shutdown() { impl_->shutdown(); }
} // namespace robot
