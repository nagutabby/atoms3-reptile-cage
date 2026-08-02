#include "switchbot_ble.h"
#include <NimBLEDevice.h>

namespace SwitchBotBLE {

const char* SERVICE_UUID       = "cba20d00-224d-11e6-9fb8-0002a5d5c51b";
const char* WRITE_CHAR_UUID    = "cba20002-224d-11e6-9fb8-0002a5d5c51b";
const char* NOTIFY_CHAR_UUID   = "cba20003-224d-11e6-9fb8-0002a5d5c51b";

namespace {

// Shared across a single sendCommand() call - NimBLE notify callbacks run on
// the BLE host task, not the calling task, so we hand data back via a
// volatile-ish holder guarded by the fact only one sendCommand runs at a time.
Response* g_pendingResponse = nullptr;

void notifyCallback(NimBLERemoteCharacteristic* /*chr*/, uint8_t* pData, size_t length, bool /*isNotify*/) {
    if (g_pendingResponse == nullptr) return;
    size_t n = length > sizeof(g_pendingResponse->data) ? sizeof(g_pendingResponse->data) : length;
    memcpy(g_pendingResponse->data, pData, n);
    g_pendingResponse->len = n;
    g_pendingResponse->ok = true;
}

} // namespace

namespace {

// Single connect/write/wait-for-RESP attempt, no retry. Returns true if the
// REQ was sent; resp.ok reflects whether a RESP notification actually arrived.
// `addrType` is BLE_ADDR_PUBLIC or BLE_ADDR_RANDOM - callers don't reliably
// know which a given device uses (some Plug Minis have shown up using a
// random static address instead of public), so sendCommand() tries both.
bool sendCommandOnce(const char* mac, const uint8_t* cmd, size_t cmdLen, Response& resp,
                     uint32_t timeoutMs, uint8_t addrType) {
    resp = Response{};

    NimBLEClient* pClient = NimBLEDevice::createClient();
    pClient->setConnectTimeout(5);

    bool connected = pClient->connect(NimBLEAddress(std::string(mac), addrType));
    if (!connected) {
        Serial.printf("[BLE] connect failed (addrType=%u): %s\n", addrType, mac);
        NimBLEDevice::deleteClient(pClient);
        return false;
    }

    bool success = false;
    NimBLERemoteService* pSvc = pClient->getService(SERVICE_UUID);
    if (pSvc == nullptr) {
        Serial.printf("[BLE] service not found: %s\n", mac);
    } else {
        NimBLERemoteCharacteristic* pWrite = pSvc->getCharacteristic(WRITE_CHAR_UUID);
        NimBLERemoteCharacteristic* pNotify = pSvc->getCharacteristic(NOTIFY_CHAR_UUID);
        if (pWrite == nullptr || pNotify == nullptr) {
            Serial.printf("[BLE] characteristic not found: %s\n", mac);
        } else {
            g_pendingResponse = &resp;
            if (pNotify->canNotify()) {
                pNotify->subscribe(true, notifyCallback);
            }

            if (pWrite->writeValue(cmd, cmdLen, true)) {
                uint32_t start = millis();
                while (!resp.ok && (millis() - start) < timeoutMs) {
                    delay(20);
                }
                success = true; // request was sent; resp.ok reflects whether a RESP arrived
            } else {
                Serial.printf("[BLE] write failed: %s\n", mac);
            }

            if (pNotify->canNotify()) {
                pNotify->unsubscribe();
            }
            g_pendingResponse = nullptr;
        }
    }

    pClient->disconnect();
    NimBLEDevice::deleteClient(pClient);
    return success;
}

} // namespace

bool sendCommand(const char* mac, const uint8_t* cmd, size_t cmdLen, Response& resp,
                  uint32_t timeoutMs, uint8_t maxAttempts, uint32_t retryDelayMs) {
    // Alternate address type each attempt so a wrong first guess doesn't
    // burn through all the retries on the same mistake.
    static const uint8_t addrTypes[] = {BLE_ADDR_PUBLIC, BLE_ADDR_RANDOM};

    for (uint8_t attempt = 1; attempt <= maxAttempts; attempt++) {
        uint8_t addrType = addrTypes[(attempt - 1) % 2];
        bool sent = sendCommandOnce(mac, cmd, cmdLen, resp, timeoutMs, addrType);
        if (sent && resp.ok) return true;

        if (attempt < maxAttempts) {
            Serial.printf("[BLE] retry %u/%u: %s\n", attempt, maxAttempts, mac);
            delay(retryDelayMs);
        }
    }
    return false;
}

bool plugTurnOn(const char* mac) {
    const uint8_t cmd[] = {0x57, 0x0F, 0x50, 0x01, 0x01, 0x80};
    Response resp;
    if (!sendCommand(mac, cmd, sizeof(cmd), resp)) return false;
    return resp.ok && resp.len >= 2 && resp.data[0] == 0x01 && resp.data[1] == 0x80;
}

bool plugTurnOff(const char* mac) {
    const uint8_t cmd[] = {0x57, 0x0F, 0x50, 0x01, 0x01, 0x00};
    Response resp;
    if (!sendCommand(mac, cmd, sizeof(cmd), resp)) return false;
    return resp.ok && resp.len >= 2 && resp.data[0] == 0x01 && resp.data[1] == 0x00;
}

bool plugReadState(const char* mac, bool& isOn) {
    const uint8_t cmd[] = {0x57, 0x0F, 0x51, 0x01};
    Response resp;
    if (!sendCommand(mac, cmd, sizeof(cmd), resp)) return false;
    if (!resp.ok || resp.len < 2 || resp.data[0] != 0x01) return false;
    isOn = (resp.data[1] == 0x80);
    return true;
}

namespace {

bool meterScanOnce(const char* mac, float& tempC, uint8_t& humidity, uint32_t scanSeconds) {
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setActiveScan(true);
    pScan->setInterval(100);
    pScan->setWindow(99);

    // NimBLEScan::start()'s duration is in seconds (it multiplies by 1000
    // internally) - passing already-converted milliseconds here previously
    // caused a 5s scan to run for ~5000s ("stuck" scanning).
    NimBLEScanResults results = pScan->start(scanSeconds, false);

    bool found = false;
    for (int i = 0; i < results.getCount() && !found; i++) {
        NimBLEAdvertisedDevice d = results.getDevice(i);
        if (!d.getAddress().equals(NimBLEAddress(std::string(mac), BLE_ADDR_PUBLIC))) continue;
        if (!d.haveManufacturerData()) continue;

        std::string mfg = d.getManufacturerData();
        // WoSensorTHO manufacturer-specific data (Type 0xFF), per meter.md "Other" section:
        //   data[10] fractional temp, data[11] sign+integer temp, data[12] humidity
        if (mfg.length() < 13) continue;
        const uint8_t* data = reinterpret_cast<const uint8_t*>(mfg.data());

        float fraction = data[10] & 0x0F;
        int sign = (data[11] & 0x80) ? 1 : -1;
        int integer = data[11] & 0x7F;
        tempC = sign * (integer + fraction * 0.1f);
        humidity = data[12] & 0x7F;
        found = true;
    }

    pScan->clearResults();
    return found;
}

} // namespace

bool meterScanRead(const char* mac, float& tempC, uint8_t& humidity, uint32_t scanSeconds, uint8_t maxAttempts) {
    for (uint8_t attempt = 1; attempt <= maxAttempts; attempt++) {
        if (meterScanOnce(mac, tempC, humidity, scanSeconds)) return true;
        if (attempt < maxAttempts) {
            Serial.printf("[Meter] retry %u/%u: %s\n", attempt, maxAttempts, mac);
        }
    }
    return false;
}

} // namespace SwitchBotBLE
