#pragma once
// Minimal SwitchBot BLE (unencrypted) helper for AtomS3.
// Protocol reference:
//   https://github.com/OpenWonderLabs/SwitchBotAPI-BLE/blob/latest/devicetypes/plugmini.md
//   https://github.com/OpenWonderLabs/SwitchBotAPI-BLE/blob/latest/devicetypes/meter.md

#include <Arduino.h>
#include <cstdint>
#include <cstddef>

namespace SwitchBotBLE {

// Common GATT service/characteristics shared by Plug Mini, Bot and Meter.
extern const char* SERVICE_UUID;
extern const char* WRITE_CHAR_UUID;  // Terminal -> Device (write REQ here)
extern const char* NOTIFY_CHAR_UUID; // Device -> Terminal (subscribe for RESP)

struct Response {
    bool ok = false;          // true if a RESP notification arrived before timeout
    uint8_t data[20] = {0};
    size_t len = 0;
};

// Connects to `mac` ("aa:bb:cc:dd:ee:ff"), writes `cmd`, waits for one
// notification on NOTIFY_CHAR_UUID, then disconnects. Returns false on any
// connection/service/characteristic failure; check resp.ok for the RESP itself.
// On failure (connect/service/characteristic/write error, or RESP timeout),
// retries up to `maxAttempts` times total, waiting `retryDelayMs` between
// attempts - BLE connections to these devices are flaky enough in practice
// that a single miss shouldn't be treated as a real failure.
bool sendCommand(const char* mac, const uint8_t* cmd, size_t cmdLen,
                  Response& resp, uint32_t timeoutMs = 3000,
                  uint8_t maxAttempts = 3, uint32_t retryDelayMs = 400);

// ---- Plug Mini (0x0F expansion command) ----
bool plugTurnOn(const char* mac);
bool plugTurnOff(const char* mac);
// Returns true and fills `isOn` on success.
bool plugReadState(const char* mac, bool& isOn);

// ---- Meter (waterproof/outdoor, WoSensorTHO) ----
// This device is read via passive BLE advertisement scanning rather than a
// GATT connection - see the "Other / Outdoor Temperature/Humidity Sensor"
// section of meter.md. Scans for `scanSeconds`, looking for an advertisement
// from `mac`. Returns true and fills tempC/humidity on success. Retries the
// whole scan up to `maxAttempts` times if the device isn't seen.
bool meterScanRead(const char* mac, float& tempC, uint8_t& humidity, uint32_t scanSeconds = 5,
                    uint8_t maxAttempts = 3);

} // namespace SwitchBotBLE
