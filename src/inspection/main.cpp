// ニシアフリカトカゲモドキ スマートケージ - 点検用ファームウェア
//
// 本番の自動制御ファームウェア (src/automation/main.cpp) とは別の、配線・機器の
// 疎通確認だけを行うためのスケッチ。以下を起動時に自動実行する(AtomS3のボタン
// 操作は不要):
//   - 防水温湿度計のパッシブスキャン
//   - UVBプラグ / ヒーター用プラグ / ミスト用プラグそれぞれの ON/OFF 切り替え
//     (ミスト用プラグは電源ONで自動的に噴射が始まる機器のため、本番と同じ
//     10秒待機してからOFFにする挙動をそのまま検証する)
//
// switchbot_ble.h/.cpp のリトライ(3回・Public/Randomアドレスタイプ交互)は
// そのまま活きるため、ここでは追加のリトライ制御は行わない。

#include <M5Unified.h>
#include <NimBLEDevice.h>
#include "switchbot_ble.h"

// =========================================================================
// 各機器のMACアドレス設定 (小文字・コロン区切り) - automation/main.cpp と同じ
// =========================================================================
const char* METER_MAC       = "eb:6b:03:06:2f:57"; // 防水温湿度計
const char* PLUG_HEATER_MAC = "ac:27:6e:40:5a:a2"; // パネルヒーター用プラグ
const char* PLUG_UVB_MAC    = "70:af:09:17:2a:d2"; // UVBライト用プラグ
const char* PLUG_MIST_MAC   = "00:00:00:00:00:00"; // TODO: ミストシステム用プラグミニ。実機到着後に実際のMACアドレスに置き換える

// ミスト継続時間は本番と同じ値にしておく(点検内容が本番と乖離しないように)。
// 電源ON直後の5秒間はウォームアップで噴射されないため、実際の噴射10秒と合わせて
// 合計15秒待ってからOFFにする。
static const uint32_t MIST_WARMUP_MS         = 5UL * 1000;
static const uint32_t MIST_SPRAY_DURATION_MS = 10UL * 1000;
static const uint32_t MIST_TOTAL_ON_MS       = MIST_WARMUP_MS + MIST_SPRAY_DURATION_MS;

namespace {

void logLine(const String& msg) {
    Serial.println(msg);
    M5.Display.println(msg);
}

void checkMeter() {
    logLine("[Meter] scanning...");
    float tempC = 0;
    uint8_t humidity = 0;
    if (SwitchBotBLE::meterScanRead(METER_MAC, tempC, humidity, 5)) {
        logLine("[Meter] OK " + String(tempC, 1) + "C " + String(humidity) + "%");
    } else {
        logLine("[Meter] NOT FOUND (scan timeout)");
    }
}

// プラグミニの状態読み取り→ON→OFFを行い、応答が期待通りか確認する。
void checkPlug(const char* name, const char* mac) {
    logLine(String("[") + name + "] read state...");
    bool isOn = false;
    if (SwitchBotBLE::plugReadState(mac, isOn)) {
        logLine(String("[") + name + "] state=" + (isOn ? "ON" : "OFF"));
    } else {
        logLine(String("[") + name + "] read FAILED");
        return;
    }

    logLine(String("[") + name + "] turn ON...");
    if (SwitchBotBLE::plugTurnOn(mac)) {
        logLine(String("[") + name + "] ON OK");
    } else {
        logLine(String("[") + name + "] ON FAILED");
    }
    delay(2000);

    logLine(String("[") + name + "] turn OFF...");
    if (SwitchBotBLE::plugTurnOff(mac)) {
        logLine(String("[") + name + "] OFF OK");
    } else {
        logLine(String("[") + name + "] OFF FAILED");
    }
}

// 本番と同じミスト挙動を検証する: 電源ON(自動で噴射開始) → 待機 → 電源OFF。
void checkMistPlug() {
    logLine("[Mist Plug] read state...");
    bool isOn = false;
    if (SwitchBotBLE::plugReadState(PLUG_MIST_MAC, isOn)) {
        logLine(String("[Mist Plug] state=") + (isOn ? "ON" : "OFF"));
    } else {
        logLine("[Mist Plug] read FAILED");
        return;
    }

    logLine("[Mist Plug] turn ON (spray should start)...");
    if (!SwitchBotBLE::plugTurnOn(PLUG_MIST_MAC)) {
        logLine("[Mist Plug] ON FAILED, aborting");
        return;
    }

    logLine("[Mist Plug] waiting " + String(MIST_TOTAL_ON_MS / 1000) + "s (5s warmup + " +
            String(MIST_SPRAY_DURATION_MS / 1000) + "s spray)...");
    delay(MIST_TOTAL_ON_MS);

    logLine("[Mist Plug] turn OFF...");
    if (SwitchBotBLE::plugTurnOff(PLUG_MIST_MAC)) {
        logLine("[Mist Plug] OFF OK");
    } else {
        logLine("[Mist Plug] OFF FAILED (spray auto-stops by itself, but power stays on)");
    }
}

void runFullCheck() {
    M5.Display.clear();
    M5.Display.setCursor(0, 0);
    logLine("=== inspection check start ===");

    checkMeter();
    checkPlug("UVB Plug", PLUG_UVB_MAC);
    checkPlug("Heater Plug", PLUG_HEATER_MAC);
    checkMistPlug();

    logLine("=== inspection check done ===");
    logLine("Press BtnA to run again");
}

} // namespace

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);

    M5.Display.setRotation(1);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);

    Serial.begin(115200);
    delay(1000);

    NimBLEDevice::init("AtomS3-ReptileCage-Inspect");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    M5.Display.clear();
    M5.Display.setCursor(0, 0);
    logLine("Reptile cage inspection");
    logLine("Auto-running at boot...");

    runFullCheck(); // 起動時に自動実行 (ボタン操作不要)
}

void loop() {
    M5.update();
    if (M5.BtnA.wasPressed()) {
        runFullCheck(); // 点検ツールなので手元での再実行用にBtnAも残す
    }
    delay(10);
}
