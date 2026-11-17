#pragma once
#include <stdint.h>
namespace board {
// Schematic U1 nets, matched to the manufactured PCB1/Y2; no Nano pins used.
constexpr uint8_t STEP[4] = {4,9,13,35};
constexpr uint8_t DIR[4] = {5,10,14,36};
constexpr uint8_t ENABLE[4] = {6,11,15,37}; // DRV8434 ENBLE: HIGH enables bridges.
constexpr uint8_t FAULT_N[4] = {7,12,16,38};
constexpr uint8_t CURRENT_PWM[4] = {8,17,18,21};
constexpr uint8_t WAKE = 47; // HIGH -> Q3 on -> Q2 off -> nSLEEP released.
constexpr uint8_t EXT_SELECTED = 2; // TPS2116 ST HIGH = external VIN1 selected.
constexpr uint8_t USB_PRESENT_N = 1;
// M1/M3 are the top pair, M2/M4 the bottom pair. Verify wheel travel on first article.
constexpr bool FORWARD_HIGH[4] = {true,false,true,false};
constexpr unsigned MICROSTEPS = 64;
constexpr int MAX_SPEED = 16000;
constexpr unsigned DEFAULT_CURRENT_MA = 500;
constexpr unsigned MAX_CURRENT_MA = 600;
constexpr unsigned FULL_SCALE_MA = 716; // Nominal 3.3V * 10k/(24.9k+10k) / 1.32(V/A).
constexpr uint32_t WATCHDOG_MS = 1000;
constexpr char BLE_NAME[] = "Propulsion-PCB";
constexpr char SERVICE_UUID[] = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
constexpr char RX_UUID[] = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
constexpr char TX_UUID[] = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
}
