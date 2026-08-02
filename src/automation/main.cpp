// ニシアフリカトカゲモドキ スマートケージ - 自動制御ファームウェア
//
// 設計根拠: docs/automation-story.md 参照。
// M5Stack AtomS3 から BLE (NimBLE-Arduino) のみ (Wi-Fi不使用) で以下4台を自律制御する:
//   - SwitchBot プラグミニ (UVBライト用) : 12時間ごとにON/OFFを反転 (タイマー制御)
//   - SwitchBot プラグミニ (パネルヒーター用) : 温度ヒステリシス制御
//   - SwitchBot プラグミニ (ミストシステム用) : 電源ONで最大10秒自動噴射して
//     停止する機器。湿度ヒステリシス + クールダウンでON→待機→OFFを自動実行する。
//   - SwitchBot 防水温湿度計 : パッシブBLEスキャンで温湿度を定期取得
//
// AtomS3本体のボタン操作は一切使用しない。起動後、処理開始前に必ず4台全てへの
// 疎通確認(+リトライ)を行い、結果をログとして表示してから自動制御ループに入る。

#include <M5Unified.h>
#include <NimBLEDevice.h>
#include "switchbot_ble.h"

// =========================================================================
// 1. 各機器のMACアドレス設定 (小文字・コロン区切り)
// =========================================================================
const char* METER_MAC       = "eb:6b:03:06:2f:57"; // 防水温湿度計
const char* PLUG_HEATER_MAC = "ac:27:6e:40:5a:a2"; // パネルヒーター用プラグ
const char* PLUG_UVB_MAC    = "70:af:09:17:2a:d2"; // UVBライト用プラグ
const char* PLUG_MIST_MAC   = "00:00:00:00:00:00"; // TODO: ミストシステム用プラグミニ。実機到着後に実際のMACアドレスに置き換える

// =========================================================================
// 2. 自動制御パラメータ (docs/automation-story.md セクション5と同じ値)
// =========================================================================
static const uint32_t UVB_CYCLE_MS                   = 12UL * 60 * 60 * 1000; // UVBサイクル長: 12時間
static const uint32_t UVB_RETRY_MS                    = 60UL * 1000;          // UVB切替失敗時の再試行間隔: 1分

static const uint32_t TEMP_HUMIDITY_CHECK_INTERVAL_MS = 5UL * 60 * 1000;      // 温湿度チェック間隔: 5分
static const float    HEATER_OFF_TEMP_C               = 32.0f;                // ヒーターOFFしきい値
static const float    HEATER_ON_TEMP_C                = 28.0f;                // ヒーターONしきい値

static const float    MIST_HUMIDITY_THRESHOLD         = 55.0f;                // ミスト開始しきい値(%RH)
static const uint32_t MIST_COOLDOWN_MS                = 2UL * 60 * 60 * 1000; // ミストクールダウン(最短噴霧間隔): 2時間
static const uint32_t MIST_WARMUP_MS                  = 5UL * 1000;           // 電源ON直後の待機(この間は噴射されない): 5秒
static const uint32_t MIST_SPRAY_DURATION_MS          = 10UL * 1000;          // 噴射時間: 電源ONで最大10秒噴射し自動停止
static const uint32_t MIST_TOTAL_ON_MS                = MIST_WARMUP_MS + MIST_SPRAY_DURATION_MS; // 電源ONからOFFまでの合計待機: 15秒

static const uint32_t SENSOR_STALE_MS                 = 30UL * 60 * 1000;     // センサー無応答の許容時間: 30分

// 起動時の疎通確認: ライブラリ側の内部リトライ(3回)に加え、機器ごとにこの回数まで
// 追加でラウンドを繰り返す。全ラウンド失敗した機器は「未接続」として記録した上で
// 自動制御ループに進み、以降の定期チェックで引き続き接続を試みる。
static const uint8_t  STARTUP_CHECK_ROUNDS            = 3;
static const uint32_t STARTUP_CHECK_RETRY_DELAY_MS    = 3000;

namespace {

void logLine(const String& msg) {
    Serial.println(msg);
    M5.Display.println(msg);
}

// ---- 内部状態 ----
bool uvbIsOn = false;
uint32_t uvbNextActionMs = 0;

bool heaterIsOn = false;

bool haveTempReading = false;
float lastTempC = 0.0f;
uint8_t lastHumidity = 0;
uint32_t lastSensorReadMs = 0;
uint32_t nextSensorCheckMs = 0;

bool haveMisted = false;
uint32_t lastMistMs = 0;
// ミストシステムは電源ONで最大10秒噴射すると自動停止するため、OFFコマンドが
// 失敗して電源が入りっぱなしになっても噴射し続ける心配はない。ただし次の
// トリガーで新たな噴射を始めるには一度電源を切っておく必要があるため、
// mistIsOn=true のまま残っていたら次回チェック時に改めてOFFを試みる。
bool mistIsOn = false;

// =========================================================================
// 3. 起動時疎通確認 (+リトライ)
// =========================================================================

bool waitForMeter(float& tempC, uint8_t& humidity) {
    for (uint8_t round = 1; round <= STARTUP_CHECK_ROUNDS; round++) {
        if (SwitchBotBLE::meterScanRead(METER_MAC, tempC, humidity)) return true;
        logLine("[Meter] connectivity check " + String(round) + "/" + String(STARTUP_CHECK_ROUNDS) + " failed");
        if (round < STARTUP_CHECK_ROUNDS) delay(STARTUP_CHECK_RETRY_DELAY_MS);
    }
    return false;
}

bool waitForPlugState(const char* name, const char* mac, bool& isOn) {
    for (uint8_t round = 1; round <= STARTUP_CHECK_ROUNDS; round++) {
        if (SwitchBotBLE::plugReadState(mac, isOn)) return true;
        logLine(String("[") + name + "] connectivity check " + String(round) + "/" + String(STARTUP_CHECK_ROUNDS) + " failed");
        if (round < STARTUP_CHECK_ROUNDS) delay(STARTUP_CHECK_RETRY_DELAY_MS);
    }
    return false;
}

// 4台全てへの疎通確認を行い、取得できた現在状態を内部状態の初期値として反映する。
// 各機器は最大 STARTUP_CHECK_ROUNDS 回まで粘るが、それでも失敗した場合は安全な
// デフォルト(OFF扱い/未取得)のまま自動制御ループに進む。
void runStartupConnectivityCheck() {
    logLine("=== startup connectivity check ===");

    float tempC = 0.0f;
    uint8_t humidity = 0;
    if (waitForMeter(tempC, humidity)) {
        lastTempC = tempC;
        lastHumidity = humidity;
        lastSensorReadMs = millis();
        haveTempReading = true;
        logLine("[Meter] OK " + String(tempC, 1) + "C " + String(humidity) + "%");
    } else {
        logLine("[Meter] NOT REACHABLE - will keep retrying");
    }

    if (waitForPlugState("UVB Plug", PLUG_UVB_MAC, uvbIsOn)) {
        logLine(String("[UVB Plug] OK state=") + (uvbIsOn ? "ON" : "OFF"));
    } else {
        logLine("[UVB Plug] NOT REACHABLE - assuming OFF");
    }

    if (waitForPlugState("Heater Plug", PLUG_HEATER_MAC, heaterIsOn)) {
        logLine(String("[Heater Plug] OK state=") + (heaterIsOn ? "ON" : "OFF"));
    } else {
        logLine("[Heater Plug] NOT REACHABLE - assuming OFF");
    }

    if (waitForPlugState("Mist Plug", PLUG_MIST_MAC, mistIsOn)) {
        logLine(String("[Mist Plug] OK state=") + (mistIsOn ? "ON" : "OFF"));
        if (mistIsOn) {
            logLine("[Mist Plug] was already ON at boot - turning OFF so the next trigger starts fresh");
            if (SwitchBotBLE::plugTurnOff(PLUG_MIST_MAC)) mistIsOn = false;
        }
    } else {
        logLine("[Mist Plug] NOT REACHABLE - will retry during operation");
    }

    logLine("=== connectivity check done, starting automation ===");

    uint32_t now = millis();
    uvbNextActionMs = now + UVB_CYCLE_MS;
    // Meterが起動時に取れていれば次回チェックまで通常間隔を空け、取れていなければ
    // ループ側で即座に再試行させる。
    nextSensorCheckMs = haveTempReading ? (now + TEMP_HUMIDITY_CHECK_INTERVAL_MS) : now;
}

// =========================================================================
// 4. ミストシーケンス (湿度ヒステリシスから呼ばれる)
// =========================================================================

// 電源ON → 待機(5秒のウォームアップ+最大10秒の噴射=合計15秒) → 電源OFF。
// ミストシステム自体が最大10秒で噴射を自動停止するため、OFFが多少失敗しても
// 噴射し続ける心配はない(次回のトリガーに備えて電源を切るだけの位置づけ)。
// 実際に噴射まで到達したかどうかを返す(クールダウン計測に使う)。
bool runMistSequence() {
    logLine("[Mist] turning ON...");

    if (!SwitchBotBLE::plugTurnOn(PLUG_MIST_MAC)) {
        logLine("[Mist] turn ON FAILED, aborting (not sprayed)");
        return false;
    }
    mistIsOn = true;

    delay(MIST_TOTAL_ON_MS);

    if (SwitchBotBLE::plugTurnOff(PLUG_MIST_MAC)) {
        mistIsOn = false;
        logLine("[Mist] OFF OK");
    } else {
        logLine("[Mist] turn OFF FAILED, will retry next cycle");
    }
    return true; // ONが成功した時点で噴射自体は行われている
}

// =========================================================================
// 5. 定期チェック本体
// =========================================================================

void checkUvbCycle(uint32_t now) {
    if ((int32_t)(now - uvbNextActionMs) < 0) return;

    bool wantOn = !uvbIsOn;
    bool ok = wantOn ? SwitchBotBLE::plugTurnOn(PLUG_UVB_MAC) : SwitchBotBLE::plugTurnOff(PLUG_UVB_MAC);
    if (ok) {
        uvbIsOn = wantOn;
        uvbNextActionMs = now + UVB_CYCLE_MS;
        logLine(String("[UVB] toggled -> ") + (uvbIsOn ? "ON" : "OFF"));
    } else {
        uvbNextActionMs = now + UVB_RETRY_MS;
        logLine("[UVB] toggle FAILED, retry in 1min");
    }
}

void applyHeaterHysteresis(float tempC) {
    if (heaterIsOn && tempC >= HEATER_OFF_TEMP_C) {
        if (SwitchBotBLE::plugTurnOff(PLUG_HEATER_MAC)) {
            heaterIsOn = false;
            logLine("[Heater] OFF (too hot: " + String(tempC, 1) + "C)");
        } else {
            logLine("[Heater] OFF command FAILED");
        }
    } else if (!heaterIsOn && tempC <= HEATER_ON_TEMP_C) {
        if (SwitchBotBLE::plugTurnOn(PLUG_HEATER_MAC)) {
            heaterIsOn = true;
            logLine("[Heater] ON (too cold: " + String(tempC, 1) + "C)");
        } else {
            logLine("[Heater] ON command FAILED");
        }
    }
}

void applyMistHysteresis(uint8_t humidity, uint32_t now) {
    bool cooldownElapsed = !haveMisted || (now - lastMistMs) >= MIST_COOLDOWN_MS;
    if (humidity < MIST_HUMIDITY_THRESHOLD && cooldownElapsed) {
        if (runMistSequence()) {
            lastMistMs = millis();
            haveMisted = true;
        }
    }
}

void checkTempHumidity(uint32_t now) {
    if ((int32_t)(now - nextSensorCheckMs) < 0) return;
    nextSensorCheckMs = now + TEMP_HUMIDITY_CHECK_INTERVAL_MS;

    // 前回ミストのOFFに失敗している場合、次のトリガーに備えて電源を切っておく。
    if (mistIsOn) {
        if (SwitchBotBLE::plugTurnOff(PLUG_MIST_MAC)) {
            mistIsOn = false;
            logLine("[Mist] OFF OK (retry)");
        }
    }

    float tempC = 0.0f;
    uint8_t humidity = 0;
    if (SwitchBotBLE::meterScanRead(METER_MAC, tempC, humidity)) {
        lastTempC = tempC;
        lastHumidity = humidity;
        lastSensorReadMs = millis();
        haveTempReading = true;
        logLine("[Meter] " + String(tempC, 1) + "C " + String(humidity) + "%");

        applyHeaterHysteresis(tempC);
        applyMistHysteresis(humidity, millis());
    } else {
        logLine("[Meter] read failed");
    }

    // フェイルセーフ: センサーが長時間無応答ならヒーターを安全側(OFF)に固定する。
    if (haveTempReading && heaterIsOn && (millis() - lastSensorReadMs) >= SENSOR_STALE_MS) {
        if (SwitchBotBLE::plugTurnOff(PLUG_HEATER_MAC)) {
            heaterIsOn = false;
            logLine("[Heater] OFF (sensor stale, fail-safe)");
        }
    }
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

    NimBLEDevice::init("AtomS3-ReptileCage");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    M5.Display.clear();
    M5.Display.setCursor(0, 0);
    logLine("Reptile cage automation");

    runStartupConnectivityCheck(); // 処理開始前に必ず疎通確認(+リトライ)を行う
}

void loop() {
    M5.update();
    uint32_t now = millis();

    checkUvbCycle(now);
    checkTempHumidity(now); // ヒーター・ミストの判定もこの中で行われる

    delay(10);
}
