// ヒョウモントカゲモドキ スマートケージ - 自動制御ファームウェア
//
// 設計根拠: docs/automation-story.md 参照。
// M5Stack AtomS3 から以下を自律制御する:
//   - SwitchBot プラグミニ (UVBライト用, BLE): 実時刻(JST)に基づき
//     7:00-18:59 ON / 19:00-6:59 OFF を切り替える (タイマー制御)
//   - Wi-Fi + NTP: 実時刻取得のため1時間ごとに接続->同期->切断する
//     (BLEとの無線干渉を避けるため、同期時以外はWi-Fiを完全にOFFにする)
//
// ヒーター・ミストシステム・温湿度計の制御コードはライトのみ制御への切替に伴い
// 無効化 (#if 0) して残している。将来再度有効化する場合はその節を参照。
//
// 起動後、処理開始前に必ずNTP時刻同期とUVBプラグへの疎通確認(+リトライ)を行い、
// 結果をログとして表示してから自動制御ループに入る。
// AtomS3本体のBtnAを押すと、その時点の同期済み時刻をログに表示する。

#include <M5Unified.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <time.h>
#include "switchbot_ble.h"
#include "wifi_config.h" // WIFI_SSID, WIFI_PASSWORD (.gitignore対象。wifi_config.h.exampleを参照)

// =========================================================================
// 1. 各機器のMACアドレス設定 (小文字・コロン区切り)
// =========================================================================
const char* PLUG_UVB_MAC    = "70:af:09:17:2a:d2"; // UVBライト用プラグ

#if 0
// ライトのみ制御への切替に伴い無効化。復活させる場合はこの節を有効化する。
const char* METER_MAC       = "eb:6b:03:06:2f:57"; // 防水温湿度計
const char* PLUG_HEATER_MAC = "ac:27:6e:40:5a:a2"; // パネルヒーター用プラグ
const char* PLUG_MIST_MAC   = "00:00:00:00:00:00"; // TODO: ミストシステム用プラグミニ。実機到着後に実際のMACアドレスに置き換える
#endif

// =========================================================================
// 2. 自動制御パラメータ
// =========================================================================

// ---- ライトスケジュール (実時刻ベース) ----
static const long     JST_OFFSET_SEC                   = 9L * 3600; // JST (UTC+9, DSTなし)
static const int      JST_DST_OFFSET_SEC                = 0;
static const char*    NTP_SERVER1                       = "ntp.nict.jp";
static const char*    NTP_SERVER2                       = "time.cloudflare.com";

static const uint32_t WIFI_CONNECT_TIMEOUT_MS           = 15UL * 1000;          // Wi-Fi接続タイムアウト: 15秒
static const uint32_t NTP_SYNC_TIMEOUT_MS               = 10UL * 1000;          // NTP同期待ちタイムアウト: 10秒
static const uint32_t NTP_RESYNC_INTERVAL_MS            = 60UL * 60 * 1000;     // NTP再同期間隔: 1時間
static const uint32_t NTP_RETRY_BASE_MS                 = 5UL * 60 * 1000;      // NTP同期失敗時の再試行間隔(指数バックオフの初期値): 5分
static const uint32_t NTP_RETRY_MAX_MS                  = 30UL * 60 * 1000;     // NTP同期失敗時の再試行間隔の上限: 30分

static const int      LIGHT_ON_HOUR                     = 7;  // ONにする時刻(この時刻を含む): 7:00
static const int      LIGHT_OFF_HOUR                    = 19; // OFFにする時刻(この時刻を含む): 19:00
static const uint32_t LIGHT_SCHEDULE_CHECK_INTERVAL_MS  = 60UL * 1000;          // スケジュール判定間隔: 1分
static const uint32_t LIGHT_RETRY_BASE_MS               = 60UL * 1000;          // 切替失敗時の再試行間隔(指数バックオフの初期値): 1分
static const uint32_t LIGHT_RETRY_MAX_MS                = 15UL * 60 * 1000;     // 切替失敗時の再試行間隔の上限: 15分

// 起動時のNTP同期・疎通確認: ライブラリ側の内部リトライ(3回)に加え、この回数まで
// 追加でラウンドを繰り返す。全ラウンド失敗した場合は「未同期/未接続」として記録した
// 上で自動制御ループに進み、以降の定期チェックで引き続き試行を続ける。
static const uint8_t  STARTUP_CHECK_ROUNDS              = 3;
static const uint32_t STARTUP_CHECK_RETRY_DELAY_MS      = 3000;      // ラウンド間待機(指数バックオフの初期値)
static const uint32_t STARTUP_CHECK_RETRY_MAX_DELAY_MS  = 15000;     // ラウンド間待機の上限

#if 0
// ライトのみ制御への切替に伴い無効化。復活させる場合はこの節を有効化する。
static const uint32_t TEMP_HUMIDITY_CHECK_INTERVAL_MS = 5UL * 60 * 1000;      // 温湿度チェック間隔: 5分
static const float    HEATER_OFF_TEMP_C               = 32.0f;                // ヒーターOFFしきい値
static const float    HEATER_ON_TEMP_C                = 28.0f;                // ヒーターONしきい値

static const float    MIST_HUMIDITY_THRESHOLD         = 55.0f;                // ミスト開始しきい値(%RH)
static const uint32_t MIST_COOLDOWN_MS                = 2UL * 60 * 60 * 1000; // ミストクールダウン(最短噴霧間隔): 2時間
static const uint32_t MIST_WARMUP_MS                  = 5UL * 1000;           // 電源ON直後の待機(この間は噴射されない): 5秒
static const uint32_t MIST_SPRAY_DURATION_MS          = 10UL * 1000;          // 噴射時間: 電源ONで最大10秒噴射し自動停止
static const uint32_t MIST_TOTAL_ON_MS                = MIST_WARMUP_MS + MIST_SPRAY_DURATION_MS; // 電源ONからOFFまでの合計待機: 15秒

static const uint32_t SENSOR_STALE_MS                 = 30UL * 60 * 1000;     // センサー無応答の許容時間: 30分
#endif

namespace {

void logLine(const String& msg) {
    Serial.println(msg);
    M5.Display.println(msg);
}

// 失敗回数(0始まり)に応じて指数的に増加する待機時間を計算する。
// baseMs * 2^failureCount を maxMs で上限する。
uint32_t backoffDelayMs(uint32_t baseMs, uint8_t failureCount, uint32_t maxMs) {
    uint8_t exponent = failureCount > 10 ? 10 : failureCount; // シフトによる桁あふれを防ぐ
    uint32_t delayMs = baseMs << exponent;
    return delayMs > maxMs ? maxMs : delayMs;
}

// ---- 内部状態 ----
bool uvbIsOn = false;
uint32_t nextNtpSyncMs = 0;
uint32_t nextLightCheckMs = 0;
bool timeSynced = false;
uint8_t ntpFailureCount = 0;
uint8_t lightRetryFailureCount = 0;

#if 0
// ライトのみ制御への切替に伴い無効化。復活させる場合はこの節を有効化する。
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
#endif

// =========================================================================
// 3. Wi-Fi + NTP 時刻同期
// =========================================================================

// Wi-Fi接続->NTP同期->切断を1回試行する。成否に関わらず必ずWi-Fiを完全にOFFにして
// 戻る(同期時以外はBLEと無線を共有しないようにするため)。
bool wifiConnectAndSyncNtp() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
        delay(200);
    }

    bool ok = false;
    if (WiFi.status() == WL_CONNECTED) {
        configTime(JST_OFFSET_SEC, JST_DST_OFFSET_SEC, NTP_SERVER1, NTP_SERVER2);
        struct tm timeinfo;
        ok = getLocalTime(&timeinfo, NTP_SYNC_TIMEOUT_MS);
    } else {
        logLine("[NTP] Wi-Fi connect FAILED");
    }

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    logLine(ok ? "[NTP] sync OK" : "[NTP] sync FAILED");
    return ok;
}

// 起動時用: STARTUP_CHECK_ROUNDS回まで粘って時刻同期を試みる。ラウンド間の待機は
// 指数バックオフで増やす。
bool syncTimeWithRetry() {
    for (uint8_t round = 1; round <= STARTUP_CHECK_ROUNDS; round++) {
        if (wifiConnectAndSyncNtp()) return true;
        logLine("[NTP] startup sync " + String(round) + "/" + String(STARTUP_CHECK_ROUNDS) + " failed");
        if (round < STARTUP_CHECK_ROUNDS) {
            delay(backoffDelayMs(STARTUP_CHECK_RETRY_DELAY_MS, round - 1, STARTUP_CHECK_RETRY_MAX_DELAY_MS));
        }
    }
    return false;
}

void checkNtpResync(uint32_t now) {
    if ((int32_t)(now - nextNtpSyncMs) < 0) return;

    bool ok = wifiConnectAndSyncNtp();
    timeSynced = timeSynced || ok; // 一度同期できていれば、直近の再同期が失敗しても古い時刻情報を使い続ける
    if (ok) {
        ntpFailureCount = 0;
        nextNtpSyncMs = now + NTP_RESYNC_INTERVAL_MS;
    } else {
        uint32_t delayMs = backoffDelayMs(NTP_RETRY_BASE_MS, ntpFailureCount, NTP_RETRY_MAX_MS);
        ntpFailureCount++;
        nextNtpSyncMs = now + delayMs;
        logLine("[NTP] retry in " + String(delayMs / 1000) + "s");
    }
}

// =========================================================================
// 4. 起動時疎通確認 (+リトライ)
// =========================================================================

bool waitForPlugState(const char* name, const char* mac, bool& isOn) {
    for (uint8_t round = 1; round <= STARTUP_CHECK_ROUNDS; round++) {
        if (SwitchBotBLE::plugReadState(mac, isOn)) return true;
        logLine(String("[") + name + "] connectivity check " + String(round) + "/" + String(STARTUP_CHECK_ROUNDS) + " failed");
        if (round < STARTUP_CHECK_ROUNDS) delay(STARTUP_CHECK_RETRY_DELAY_MS);
    }
    return false;
}

#if 0
// ライトのみ制御への切替に伴い無効化。復活させる場合はこの節を有効化する。
bool waitForMeter(float& tempC, uint8_t& humidity) {
    for (uint8_t round = 1; round <= STARTUP_CHECK_ROUNDS; round++) {
        if (SwitchBotBLE::meterScanRead(METER_MAC, tempC, humidity)) return true;
        logLine("[Meter] connectivity check " + String(round) + "/" + String(STARTUP_CHECK_ROUNDS) + " failed");
        if (round < STARTUP_CHECK_ROUNDS) delay(STARTUP_CHECK_RETRY_DELAY_MS);
    }
    return false;
}
#endif

// UVBプラグへの疎通確認を行い、取得できた現在状態を内部状態の初期値として反映する。
// 失敗した場合は安全なデフォルト(OFF扱い)のまま自動制御ループに進む。
void runStartupConnectivityCheck() {
    logLine("=== startup connectivity check ===");

    if (waitForPlugState("UVB Plug", PLUG_UVB_MAC, uvbIsOn)) {
        logLine(String("[UVB Plug] OK state=") + (uvbIsOn ? "ON" : "OFF"));
    } else {
        logLine("[UVB Plug] NOT REACHABLE - assuming OFF");
    }

#if 0
    // ライトのみ制御への切替に伴い無効化。復活させる場合はこの節を有効化する。
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

    nextSensorCheckMs = haveTempReading ? (millis() + TEMP_HUMIDITY_CHECK_INTERVAL_MS) : millis();
#endif

    logLine("=== connectivity check done, starting automation ===");

    nextLightCheckMs = millis();
}

#if 0
// =========================================================================
// 5. ミストシーケンス (湿度ヒステリシスから呼ばれる)
// ライトのみ制御への切替に伴い無効化。復活させる場合はこの節を有効化する。
// =========================================================================

// 電源ON -> 待機(5秒のウォームアップ+最大10秒の噴射=合計15秒) -> 電源OFF。
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
#endif

// =========================================================================
// 6. ライトスケジュール判定本体
// =========================================================================

bool computeDesiredLightOn(const struct tm& t) {
    return t.tm_hour >= LIGHT_ON_HOUR && t.tm_hour < LIGHT_OFF_HOUR;
}

void checkLightSchedule(uint32_t now) {
    if ((int32_t)(now - nextLightCheckMs) < 0) return;

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 10)) {
        // まだ時刻が同期できていない。次のNTP同期を待つ(バックオフ対象外の固定間隔)。
        nextLightCheckMs = now + LIGHT_RETRY_BASE_MS;
        return;
    }

    bool wantOn = computeDesiredLightOn(timeinfo);
    if (wantOn == uvbIsOn) {
        lightRetryFailureCount = 0;
        nextLightCheckMs = now + LIGHT_SCHEDULE_CHECK_INTERVAL_MS;
        return;
    }

    bool ok = wantOn ? SwitchBotBLE::plugTurnOn(PLUG_UVB_MAC) : SwitchBotBLE::plugTurnOff(PLUG_UVB_MAC);
    if (ok) {
        uvbIsOn = wantOn;
        lightRetryFailureCount = 0;
        nextLightCheckMs = now + LIGHT_SCHEDULE_CHECK_INTERVAL_MS;
        logLine(String("[Light] toggled -> ") + (uvbIsOn ? "ON" : "OFF"));
    } else {
        uint32_t delayMs = backoffDelayMs(LIGHT_RETRY_BASE_MS, lightRetryFailureCount, LIGHT_RETRY_MAX_MS);
        lightRetryFailureCount++;
        nextLightCheckMs = now + delayMs;
        logLine("[Light] toggle FAILED, retry in " + String(delayMs / 1000) + "s");
    }
}

// BtnA押下時に、直近で同期した現在時刻をログに表示する。
void showCurrentTime() {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 10)) {
        char buf[24];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
        logLine(String("[Time] ") + buf);
    } else {
        logLine("[Time] not synced yet");
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

    M5.Display.clear();
    M5.Display.setCursor(0, 0);
    logLine("Reptile cage automation");

    // Wi-FiとBLEの無線競合を避けるため、BLE初期化前に起動時のNTP同期を済ませる。
    timeSynced = syncTimeWithRetry();
    uint32_t now = millis();
    nextNtpSyncMs = now + (timeSynced ? NTP_RESYNC_INTERVAL_MS : NTP_RETRY_BASE_MS);
    if (!timeSynced) {
        logLine("[NTP] NOT SYNCED - will keep retrying");
    }

    NimBLEDevice::init("AtomS3-ReptileCage");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    runStartupConnectivityCheck(); // 処理開始前に必ず疎通確認(+リトライ)を行う
}

void loop() {
    M5.update();
    uint32_t now = millis();

    checkNtpResync(now);
    checkLightSchedule(now);
    // checkTempHumidity(now); // ヒーター・ミストの判定。ライトのみ制御への切替に伴い無効化 (上記#if 0参照)

    if (M5.BtnA.wasPressed()) {
        showCurrentTime();
    }

    delay(10);
}
