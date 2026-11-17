#include "input_source.hpp"

#import <Foundation/Foundation.h>
#import <GameController/GameController.h>
#import <IOKit/hid/IOHIDManager.h>
#import <IOKit/hidsystem/IOHIDLib.h>

#include <atomic>
#include <cmath>
#include <memory>
#include <string>

namespace mac_control {
namespace {

CFMutableDictionaryRef make_hid_match(int usage) {
    CFMutableDictionaryRef match = CFDictionaryCreateMutable(
        kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);
    int page = kHIDPage_GenericDesktop;
    CFNumberRef page_number = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &page);
    CFNumberRef usage_number = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &usage);
    CFDictionarySetValue(match, CFSTR(kIOHIDDeviceUsagePageKey), page_number);
    CFDictionarySetValue(match, CFSTR(kIOHIDDeviceUsageKey), usage_number);
    CFRelease(page_number);
    CFRelease(usage_number);
    return match;
}

double normalized_hid_value(IOHIDElementRef element, IOHIDValueRef value) {
    const CFIndex minimum = IOHIDElementGetLogicalMin(element);
    const CFIndex maximum = IOHIDElementGetLogicalMax(element);
    if (maximum <= minimum) return 0.0;
    const double raw = static_cast<double>(IOHIDValueGetIntegerValue(value));
    const double normalized = 2.0 * (raw - static_cast<double>(minimum)) /
                                  static_cast<double>(maximum - minimum) -
                              1.0;
    return std::max(-1.0, std::min(1.0, normalized));
}

class MacOSGamepad final : public InputSource {
public:
    MacOSGamepad(int axis_usage, int stop_button_usage)
        : axis_usage_(axis_usage), stop_button_usage_(stop_button_usage) {
        // 本程序是命令行进程，没有 AppKit 自带的事件循环。允许后台读取，
        // 并在 poll() 中主动处理主线程事件，才能收到异步连接结果。
        if (@available(macOS 11.0, *)) {
            [GCController setShouldMonitorBackgroundEvents:YES];
        }
        // 启动蓝牙手柄发现；USB 手柄通常会直接出现在 controllers 列表。
        [GCController startWirelessControllerDiscoveryWithCompletionHandler:^{}];

        // 部分第三方手柄会被 macOS 识别为标准 HID Gamepad，却不出现在
        // GCController.controllers 中。仅匹配 Gamepad/Joystick，避免读取键盘等设备。
        hid_access_ = IOHIDCheckAccess(kIOHIDRequestTypeListenEvent);
        if (hid_access_ == kIOHIDAccessTypeUnknown) {
            (void)IOHIDRequestAccess(kIOHIDRequestTypeListenEvent);
            hid_access_ = IOHIDCheckAccess(kIOHIDRequestTypeListenEvent);
        }
        hid_manager_ = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDManagerOptionNone);
        CFMutableDictionaryRef gamepad = make_hid_match(kHIDUsage_GD_GamePad);
        CFMutableDictionaryRef joystick = make_hid_match(kHIDUsage_GD_Joystick);
        const void *matches[] = {gamepad, joystick};
        CFArrayRef matching_array = CFArrayCreate(
            kCFAllocatorDefault, matches, 2, &kCFTypeArrayCallBacks);
        IOHIDManagerSetDeviceMatchingMultiple(hid_manager_, matching_array);
        IOHIDManagerScheduleWithRunLoop(
            hid_manager_, CFRunLoopGetMain(), kCFRunLoopDefaultMode);
        hid_open_result_ = IOHIDManagerOpen(hid_manager_, kIOHIDOptionsTypeNone);
        CFRelease(matching_array);
        CFRelease(gamepad);
        CFRelease(joystick);
    }

    ~MacOSGamepad() override {
        [GCController stopWirelessControllerDiscovery];
        if (hid_manager_ != nullptr) {
            IOHIDManagerClose(hid_manager_, kIOHIDOptionsTypeNone);
            IOHIDManagerUnscheduleFromRunLoop(
                hid_manager_, CFRunLoopGetMain(), kCFRunLoopDefaultMode);
            CFRelease(hid_manager_);
        }
    }

    InputSample poll() override {
        @autoreleasepool {
            InputSample sample;

            // GameController 的连接检测不是同步完成的，连接通知投递在主线程。
            // 普通 macOS App 会由 AppKit 处理这里的事件；命令行程序需主动处理。
            [[NSRunLoop mainRunLoop]
                runMode:NSDefaultRunLoopMode
                beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.001]];

            NSArray<GCController *> *controllers = [GCController controllers];
            if (controllers.count > 0) {
                GCController *controller = controllers.firstObject;
                GCExtendedGamepad *gamepad = controller.extendedGamepad;
                connected_.store(true);
                raw_hid_.store(false);
                if (gamepad != nil) {
                // 兼容原 IOHID 参数：X/Y 对应左摇杆，Rx/Ry 对应右摇杆。
                    switch (axis_usage_) {
                    case 0x30: sample.propulsion_axis = gamepad.leftThumbstick.xAxis.value; break;
                    case 0x31: sample.propulsion_axis = -gamepad.leftThumbstick.yAxis.value; break;
                    case 0x34: sample.propulsion_axis = -gamepad.rightThumbstick.yAxis.value; break;
                    case 0x33:
                    default: sample.propulsion_axis = gamepad.rightThumbstick.xAxis.value; break;
                    }

                    // 默认 usage 7 映射为 Options/Back；其他值便于真机调试常用按键。
                    switch (stop_button_usage_) {
                    case 1: sample.emergency_stop = gamepad.buttonA.pressed; break;
                    case 2: sample.emergency_stop = gamepad.buttonB.pressed; break;
                    case 3: sample.emergency_stop = gamepad.buttonX.pressed; break;
                    case 4: sample.emergency_stop = gamepad.buttonY.pressed; break;
                    case 8: sample.emergency_stop = gamepad.buttonMenu.pressed; break;
                    case 7:
                    default: sample.emergency_stop = gamepad.buttonOptions.pressed; break;
                    }
                } else if (controller.microGamepad != nil) {
                    sample.propulsion_axis = controller.microGamepad.dpad.xAxis.value;
                    sample.emergency_stop = controller.microGamepad.buttonX.pressed;
                }
                return sample;
            }

            if (poll_raw_hid(sample)) return sample;
            connected_.store(false);
            raw_hid_.store(false);
            return sample;
        }
    }

    std::string description() const override {
        if (!connected_.load() && hid_access_ != kIOHIDAccessTypeGranted) {
            return "macOS 标准 HID 手柄（需要输入监控权限）";
        }
        if (!connected_.load()) return "macOS 手柄（未检测到设备）";
        return raw_hid_.load() ? "macOS 标准 HID 手柄（已连接）"
                               : "macOS GameController 手柄（已连接）";
    }

    bool connected() const override { return connected_.load(); }

private:
    bool poll_raw_hid(InputSample &sample) {
        if (hid_manager_ == nullptr || hid_open_result_ != kIOReturnSuccess) return false;
        CFSetRef devices = IOHIDManagerCopyDevices(hid_manager_);
        if (devices == nullptr || CFSetGetCount(devices) == 0) {
            if (devices != nullptr) CFRelease(devices);
            return false;
        }

        const CFIndex device_count = CFSetGetCount(devices);
        std::unique_ptr<const void *[]> device_values(new const void *[device_count]);
        CFSetGetValues(devices, device_values.get());
        bool read_axis = false;

        for (CFIndex device_index = 0; device_index < device_count && !read_axis; ++device_index) {
            IOHIDDeviceRef device =
                reinterpret_cast<IOHIDDeviceRef>(const_cast<void *>(device_values[device_index]));
            CFArrayRef elements = IOHIDDeviceCopyMatchingElements(
                device, nullptr, kIOHIDOptionsTypeNone);
            if (elements == nullptr) continue;

            // T4n Lite 的 Xbox 兼容描述符用 Z(0x32) 表示右摇杆水平轴；
            // 仍优先查找用户指定的 usage，以兼容其他标准手柄。
            const uint32_t fallback_axis = axis_usage_ == 0x33 ? 0x32 :
                                           axis_usage_ == 0x34 ? 0x35 : 0;
            IOHIDElementRef fallback_element = nullptr;
            const CFIndex element_count = CFArrayGetCount(elements);
            for (CFIndex i = 0; i < element_count; ++i) {
                IOHIDElementRef element = reinterpret_cast<IOHIDElementRef>(
                    const_cast<void *>(CFArrayGetValueAtIndex(elements, i)));
                const uint32_t page = IOHIDElementGetUsagePage(element);
                const uint32_t usage = IOHIDElementGetUsage(element);
                const IOHIDElementType type = IOHIDElementGetType(element);
                if (type != kIOHIDElementTypeInput_Misc &&
                    type != kIOHIDElementTypeInput_Button &&
                    type != kIOHIDElementTypeInput_Axis) {
                    continue;
                }

                if (page == kHIDPage_Button && usage == static_cast<uint32_t>(stop_button_usage_)) {
                    IOHIDValueRef value = nullptr;
                    if (IOHIDDeviceGetValue(device, element, &value) == kIOReturnSuccess &&
                        value != nullptr) {
                        sample.emergency_stop = IOHIDValueGetIntegerValue(value) != 0;
                    }
                }
                if (page != kHIDPage_GenericDesktop) continue;
                if (usage == fallback_axis) fallback_element = element;
                if (usage != static_cast<uint32_t>(axis_usage_)) continue;

                IOHIDValueRef value = nullptr;
                if (IOHIDDeviceGetValue(device, element, &value) == kIOReturnSuccess &&
                    value != nullptr) {
                    sample.propulsion_axis = normalized_hid_value(element, value);
                    read_axis = true;
                }
            }

            if (!read_axis && fallback_element != nullptr) {
                IOHIDValueRef value = nullptr;
                if (IOHIDDeviceGetValue(device, fallback_element, &value) == kIOReturnSuccess &&
                    value != nullptr) {
                    sample.propulsion_axis = normalized_hid_value(fallback_element, value);
                    read_axis = true;
                }
            }
            CFRelease(elements);
        }
        CFRelease(devices);

        if (read_axis) {
            connected_.store(true);
            raw_hid_.store(true);
        }
        return read_axis;
    }

    std::atomic<bool> connected_{false};
    std::atomic<bool> raw_hid_{false};
    int axis_usage_ = 0x33;
    int stop_button_usage_ = 7;
    IOHIDManagerRef hid_manager_ = nullptr;
    IOReturn hid_open_result_ = kIOReturnError;
    IOHIDAccessType hid_access_ = kIOHIDAccessTypeUnknown;
};

} // namespace

std::unique_ptr<InputSource> make_macos_gamepad(int axis_usage, int stop_button_usage) {
    return std::make_unique<MacOSGamepad>(axis_usage, stop_button_usage);
}

} // namespace mac_control
