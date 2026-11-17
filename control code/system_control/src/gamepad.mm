#include "robot.hpp"
#import <Foundation/Foundation.h>
#import <GameController/GameController.h>
#import <IOKit/hid/IOHIDManager.h>
#import <IOKit/hidsystem/IOHIDLib.h>
#include <algorithm>

namespace robot {
namespace {
CFMutableDictionaryRef match(int usage) {
    auto d = CFDictionaryCreateMutable(nullptr, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    int page = kHIDPage_GenericDesktop;
    auto p = CFNumberCreate(nullptr, kCFNumberIntType, &page);
    auto u = CFNumberCreate(nullptr, kCFNumberIntType, &usage);
    CFDictionarySetValue(d, CFSTR(kIOHIDDeviceUsagePageKey), p);
    CFDictionarySetValue(d, CFSTR(kIOHIDDeviceUsageKey), u);
    CFRelease(p); CFRelease(u); return d;
}
class Gamepad final : public InputSource {
    GCController* selected_ = nil;
    IOHIDManagerRef manager_ = nullptr;
    IOHIDDeviceRef device_ = nullptr;
    bool hid_started_ = false;
    std::string name_ = "waiting for one Bluetooth/USB gamepad";
    void start_hid() {
        hid_started_ = true;
        if (IOHIDCheckAccess(kIOHIDRequestTypeListenEvent) == kIOHIDAccessTypeUnknown)
            IOHIDRequestAccess(kIOHIDRequestTypeListenEvent);
        manager_ = IOHIDManagerCreate(nullptr, kIOHIDManagerOptionNone);
        auto pad = match(kHIDUsage_GD_GamePad), joy = match(kHIDUsage_GD_Joystick);
        const void* entries[] = {pad, joy};
        auto list = CFArrayCreate(nullptr, entries, 2, &kCFTypeArrayCallBacks);
        IOHIDManagerSetDeviceMatchingMultiple(manager_, list);
        IOHIDManagerScheduleWithRunLoop(manager_, CFRunLoopGetMain(), kCFRunLoopDefaultMode);
        IOHIDManagerOpen(manager_, kIOHIDOptionsTypeNone);
        CFRelease(list); CFRelease(pad); CFRelease(joy);
    }
    bool raw(int page, int usage, double& value, bool axis) {
        auto elements = IOHIDDeviceCopyMatchingElements(device_, nullptr, kIOHIDOptionsTypeNone);
        if (!elements) return false;
        bool found = false;
        for (CFIndex i = 0; i < CFArrayGetCount(elements); ++i) {
            auto e = static_cast<IOHIDElementRef>(const_cast<void*>(CFArrayGetValueAtIndex(elements, i)));
            auto type = IOHIDElementGetType(e);
            if (type != kIOHIDElementTypeInput_Misc && type != kIOHIDElementTypeInput_Button && type != kIOHIDElementTypeInput_Axis) continue;
            if (IOHIDElementGetUsagePage(e) != static_cast<uint32_t>(page) || IOHIDElementGetUsage(e) != static_cast<uint32_t>(usage)) continue;
            IOHIDValueRef v = nullptr;
            if (IOHIDDeviceGetValue(device_, e, &v) != kIOReturnSuccess || !v) continue;
            value = static_cast<double>(IOHIDValueGetIntegerValue(v));
            if (axis) {
                double lo = IOHIDElementGetLogicalMin(e), hi = IOHIDElementGetLogicalMax(e);
                if (hi <= lo) continue;
                value = std::clamp(2 * (value - lo) / (hi - lo) - 1, -1.0, 1.0);
            }
            found = true; break;
        }
        CFRelease(elements); return found;
    }
public:
    Gamepad() {
        if (@available(macOS 11.0, *)) [GCController setShouldMonitorBackgroundEvents:YES];
        [GCController startWirelessControllerDiscoveryWithCompletionHandler:^{}];
    }
    ~Gamepad() override {
        [GCController stopWirelessControllerDiscovery];
        if (device_) CFRelease(device_);
        if (manager_) {
            IOHIDManagerClose(manager_, kIOHIDOptionsTypeNone);
            IOHIDManagerUnscheduleFromRunLoop(manager_, CFRunLoopGetMain(), kCFRunLoopDefaultMode);
            CFRelease(manager_);
        }
    }
    Input poll() override {
        @autoreleasepool {
            Input out; out.connected = false;
            [[NSRunLoop mainRunLoop] runMode:NSDefaultRunLoopMode beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.001]];
            auto controllers = [GCController controllers];
            if (!device_ && !selected_ && controllers.count == 1 && controllers.firstObject.extendedGamepad)
                selected_ = controllers.firstObject;
            if (selected_) {
                // Do not silently switch to another controller or backend during operation.
                if (![controllers containsObject:selected_]) return out;
                auto g = selected_.extendedGamepad;
                if (!g || !g.buttonOptions) return out;
                out.connected = true;
                out.propulsion = g.rightThumbstick.xAxis.value;
                out.cylinders = {g.leftThumbstick.xAxis.value, -g.leftThumbstick.yAxis.value, -g.rightThumbstick.yAxis.value};
                out.enable = g.buttonA.pressed; out.held = g.leftShoulder.pressed; out.stop = g.buttonOptions.pressed;
                const char* vendor = selected_.vendorName.UTF8String;
                name_ = "GameController: " + std::string(vendor ? vendor : "gamepad");
                return out;
            }
            if (controllers.count > 1) { name_ = "multiple controllers: keep only the intended gamepad connected"; return out; }
            if (!hid_started_) start_hid();
            auto devices = manager_ ? IOHIDManagerCopyDevices(manager_) : nullptr;
            if (!devices) return out;
            if (!device_ && CFSetGetCount(devices) == 1) {
                const void* value = nullptr; CFSetGetValues(devices, &value);
                device_ = static_cast<IOHIDDeviceRef>(const_cast<void*>(value)); CFRetain(device_);
            }
            bool present = device_ && CFSetContainsValue(devices, device_);
            CFRelease(devices);
            if (!present) return out;
            double x, y, rx, ry, enable, held, stop;
            bool valid = raw(kHIDPage_GenericDesktop, 0x30, x, true) && raw(kHIDPage_GenericDesktop, 0x31, y, true) &&
                (raw(kHIDPage_GenericDesktop, 0x33, rx, true) || raw(kHIDPage_GenericDesktop, 0x32, rx, true)) &&
                (raw(kHIDPage_GenericDesktop, 0x34, ry, true) || raw(kHIDPage_GenericDesktop, 0x35, ry, true)) &&
                raw(kHIDPage_Button, 1, enable, false) && raw(kHIDPage_Button, 5, held, false) && raw(kHIDPage_Button, 7, stop, false);
            if (!valid) { name_ = "HID mapping incomplete; motion disabled"; return out; }
            out.connected = true; out.propulsion = rx; out.cylinders = {x, y, ry};
            out.enable = enable != 0; out.held = held != 0; out.stop = stop != 0;
            name_ = "HID: axes X/Y/Rx/Ry (Z/Rz fallback), buttons 1/5/7; verify signs in simulation";
            return out;
        }
    }
    std::string description() const override { return name_; }
};
}
std::unique_ptr<InputSource> gamepad() { return std::make_unique<Gamepad>(); }
} // namespace robot
