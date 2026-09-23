// ヒョウモントカゲモドキ スマートケージ - 自動制御ファームウェア
//
// 設計根拠: docs/automation-story.md 参照。
// M5Stack AtomS3 から以下を自律制御する:
//   - SwitchBot プラグミニ (UVBライト用, BLE): 実時刻(JST)に基づき
//     7:00-18:59 ON / 19:00-6:59 OFF を切り替える (タイマー制御)
//   - SwitchBot プラグミニ (パネルヒーター用, BLE): 温湿度チェックと同じ1分おきに、
//     直近の温度がTEMP_MAX_Cを超えていればOFF、下回っていればONにする(単純な閾値制御、
//     ヒステリシス無し)。既に同じ状態ならBLEコマンドは送らない(ライト制御と同様)。
//     状態が変化していれば都度バックエンドに報告する
//   - SwitchBot 防水温湿度計 (BLEパッシブスキャン): 1分おきに温湿度を取得し、
//     取得できたら都度Wi-Fi経由でFastAPIバックエンドに送信する
//     (本体内のデータ記録自体が1分単位のため、この間隔に合わせている)
//   - Wi-Fi + NTP: 起動時に接続し、以後は切断せず常時接続を維持する
//     (切断が続いた場合は指数バックオフで再接続を試みる)。実時刻は1時間
//     ごとに再同期する
//
// Wi-Fiを常時接続にしているため、BLE(プラグ制御・温湿度計スキャン)とは
// 常に無線を共有する。干渉でBLE側が不安定になる場合は接続維持方式を見直すこと。
//
// ヒーター・ミストシステムの制御コードはライトのみ制御への切替に伴い
// 無効化 (#if 0) して残している。将来再度有効化する場合はその節を参照。
//
// 起動後、処理開始前に必ずNTP時刻同期とUVBプラグへの疎通確認(+リトライ)を行い、
// 結果をログとして表示してから自動制御ループに入る。
// AtomS3本体のBtnAを押すと、その時点の同期済み時刻をログに表示する。
//
// 画面は常時点灯させず、書き込み直後(起動時)またはBtnA押下時にのみ5秒間点灯し、
// その後自動的に消灯する。

#include <M5Unified.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <esp_netif.h>
#include <time.h>
#include "control_logic.h"
#include "switchbot_ble.h"
#include "wifi_config.h" // WIFI_SSID, WIFI_PASSWORD, API_ENDPOINT_URL, API_KEY (.gitignore対象。wifi_config.h.exampleを参照)

// =========================================================================
// 1. 各機器のMACアドレス設定 (小文字・コロン区切り)
// =========================================================================
const char* PLUG_UVB_MAC    = "70:af:09:17:2a:d2"; // UVBライト用プラグ
const char* METER_MAC       = "eb:6b:03:06:2f:57"; // 防水温湿度計
const char* PLUG_HEATER_MAC = "ac:27:6e:40:5a:a2"; // パネルヒーター用プラグ

#if 0
// ライトのみ制御への切替に伴い無効化。復活させる場合はこの節を有効化する。
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

// ---- DNS (ルーターのDHCPが配布するDNSではなくCloudflareを使う) ----
static const uint32_t DNS_PRIMARY   = ESP_IP4TOADDR(1, 1, 1, 1);
static const uint32_t DNS_SECONDARY = ESP_IP4TOADDR(1, 0, 0, 1);

static const uint32_t WIFI_CONNECT_TIMEOUT_MS           = 15UL * 1000;          // Wi-Fi接続タイムアウト: 15秒
static const uint32_t NTP_SYNC_TIMEOUT_MS               = 10UL * 1000;          // NTP同期待ちタイムアウト: 10秒
static const uint32_t NTP_RESYNC_INTERVAL_MS            = 60UL * 60 * 1000;     // NTP再同期間隔: 1時間
static const uint32_t NTP_RETRY_BASE_MS                 = 5UL * 60 * 1000;      // NTP同期失敗時の再試行間隔(指数バックオフの初期値): 5分
static const uint32_t NTP_RETRY_MAX_MS                  = 30UL * 60 * 1000;     // NTP同期失敗時の再試行間隔の上限: 30分

static const uint32_t WIFI_RETRY_BASE_MS                = 30UL * 1000;          // Wi-Fi切断検知時の再接続間隔(指数バックオフの初期値): 30秒
static const uint32_t WIFI_RETRY_MAX_MS                 = 10UL * 60 * 1000;     // Wi-Fi再接続間隔の上限: 10分

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

// ---- 温湿度計測・レポート送信 ----
// 計測(BLE)・送信(Wi-Fi)とも1分おき(温湿度計本体のデータ記録間隔に合わせる)。
// 取得できたら都度送信する(バッファしない)。
static const uint32_t TEMP_HUMIDITY_CHECK_INTERVAL_MS  = 1UL * 60 * 1000;     // 温湿度チェック間隔: 1分
static const uint32_t METER_RETRY_BASE_MS              = 60UL * 1000;         // スキャン/送信失敗時の再試行間隔(指数バックオフの初期値): 1分
static const uint32_t METER_RETRY_MAX_MS               = 15UL * 60 * 1000;    // 再試行間隔の上限: 15分

// ---- ヒーター自動制御 ----
// この温度を境に単純なON/OFF制御を行う(ヒステリシス無し。閾値付近で温度が細かく
// 上下すると頻繁に切り替わりうるが、要件通りの単純な閾値制御とする)。
// backend/app/config.py の TEMP_MAX_C と一致させること。
static const float    TEMP_MAX_C                       = 32.0f;

// ---- 画面点灯 ----
// 書き込み直後(起動時)またはBtnA押下時のみ5秒間点灯し、以後は消灯する。
static const uint32_t DISPLAY_ON_DURATION_MS           = 5UL * 1000;          // 点灯時間: 5秒
static const uint8_t  DISPLAY_BRIGHTNESS                = 100;                // 点灯時の輝度

#if 0
// ライトのみ制御への切替に伴い無効化。復活させる場合はこの節を有効化する。
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

// 画面下端まで描画済みなら、新しい行を書く前に画面をクリアして先頭に戻す。
// (パネルのハードウェアスクロールは一部端末で描画崩れが出るため使わない)
void logLine(const String& msg) {
    Serial.println(msg);
    if (M5.Display.getCursorY() + M5.Display.fontHeight() > M5.Display.height()) {
        M5.Display.clear();
        M5.Display.setCursor(0, 0);
    }
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
uint32_t nextSensorCheckMs = 0;
uint8_t meterFailureCount = 0;
bool wifiConnected = false;

bool heaterIsOn = false;

// バックエンドに最後に報告した値。ライト・ヒーターとも、この値と実際の状態が
// 異なる(または起動後まだ一度も報告していない)場合にだけ都度の送信対象に含める。
bool lightReportKnown = false;
bool lastReportedLightOn = false;
bool heaterReportKnown = false;
bool lastReportedHeaterOn = false;
uint8_t wifiFailureCount = 0;
uint32_t nextWifiRetryMs = 0;
bool displayIsOn = false;
uint32_t displayOffAtMs = 0;

// 画面を点灯し、DISPLAY_ON_DURATION_MS後に消灯するタイマーをセットする。
void turnDisplayOn(uint32_t now) {
    M5.Display.setBrightness(DISPLAY_BRIGHTNESS);
    displayIsOn = true;
    displayOffAtMs = now + DISPLAY_ON_DURATION_MS;
}

void turnDisplayOff() {
    if (!displayIsOn) return;
    M5.Display.setBrightness(0);
    displayIsOn = false;
}

void checkDisplayTimeout(uint32_t now) {
    if (displayIsOn && (int32_t)(now - displayOffAtMs) >= 0) {
        turnDisplayOff();
    }
}

#if 0
// ライトのみ制御への切替に伴い無効化。復活させる場合はこの節を有効化する。
bool heaterIsOn = false;

bool haveTempReading = false;
float lastTempC = 0.0f;
uint8_t lastHumidity = 0;
uint32_t lastSensorReadMs = 0;

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

// IP/ゲートウェイ/サブネットはDHCP任せのまま、DNSだけ上書きする。
// (WiFi.config()でDNSを指定するにはIPも静的指定する必要があり、DHCPと両立しないため)
void overrideDns() {
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == nullptr) return;

    esp_netif_dns_info_t dns;
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4.addr = DNS_PRIMARY;
    esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns);
    dns.ip.u_addr.ip4.addr = DNS_SECONDARY;
    esp_netif_set_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns);
}

// Wi-Fi接続を1回試行する。常時接続を維持する方針のため、呼び出し側で切断は行わない。
bool wifiConnect() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
        delay(200);
    }
    bool connected = WiFi.status() == WL_CONNECTED;
    if (connected) overrideDns(); // DHCPのDNS配布より後に上書きする
    return connected;
}

// 温湿度・ライト/ヒーター状態をFastAPIバックエンドにレポート送信する(Wi-Fi接続済みで
// あることが前提)。ライト/ヒーターは値が変化した回だけ呼び出し側からhaveLight/haveHeater=
// trueで渡され、その場合だけJSONにフィールドを含める(それ以外はDB側でNULLのままにする)。
// 証明書検証は setInsecure() で省略している(組み込み機器でのCA証明書管理コストを
// 避ける簡易実装。ホビー用途として許容し、MITMリスクは認識のうえ受容する)。
bool sendReadingHttp(float tempC, uint8_t humidity, bool haveLight, bool lightOn, bool haveHeater, bool heaterOn) {
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;

    bool ok = false;
    if (http.begin(client, API_ENDPOINT_URL)) {
        http.addHeader("Content-Type", "application/json");
        http.addHeader("X-API-Key", API_KEY);
        std::string bodyStd = ControlLogic::buildReadingRequestBody(tempC, humidity, haveLight, lightOn, haveHeater, heaterOn);
        String body = String(bodyStd.c_str());
        int code = http.POST(body);
        ok = (code >= 200 && code < 300);
        logLine(ok ? "[Report] sent OK" : ("[Report] FAILED (HTTP " + String(code) + ")"));
        http.end();
    } else {
        logLine("[Report] http.begin FAILED");
    }
    return ok;
}

// 起動時用: STARTUP_CHECK_ROUNDS回まで粘ってWi-Fi接続を試みる。ラウンド間の待機は
// 指数バックオフで増やす。
bool connectWifiWithRetry() {
    for (uint8_t round = 1; round <= STARTUP_CHECK_ROUNDS; round++) {
        if (wifiConnect()) return true;
        logLine("[WiFi] startup connect " + String(round) + "/" + String(STARTUP_CHECK_ROUNDS) + " failed");
        if (round < STARTUP_CHECK_ROUNDS) {
            delay(backoffDelayMs(STARTUP_CHECK_RETRY_DELAY_MS, round - 1, STARTUP_CHECK_RETRY_MAX_DELAY_MS));
        }
    }
    return false;
}

// Wi-Fiが切断されていたら再接続を試みる(常時接続を維持する)。失敗時は指数バックオフ
// で再試行間隔を空ける。
void ensureWifiConnected(uint32_t now) {
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        wifiFailureCount = 0;
        return;
    }

    wifiConnected = false;
    if ((int32_t)(now - nextWifiRetryMs) < 0) return;

    if (wifiConnect()) {
        wifiConnected = true;
        wifiFailureCount = 0;
        logLine("[WiFi] reconnected");
    } else {
        uint32_t delayMs = backoffDelayMs(WIFI_RETRY_BASE_MS, wifiFailureCount, WIFI_RETRY_MAX_MS);
        wifiFailureCount++;
        nextWifiRetryMs = now + delayMs;
        logLine("[WiFi] reconnect FAILED, retry in " + String(delayMs / 1000) + "s");
    }
}

// NTP同期を1回試行する(Wi-Fi接続済みであることが前提)。
bool syncNtpOnce() {
    configTime(JST_OFFSET_SEC, JST_DST_OFFSET_SEC, NTP_SERVER1, NTP_SERVER2);
    struct tm timeinfo;
    bool ok = getLocalTime(&timeinfo, NTP_SYNC_TIMEOUT_MS);
    logLine(ok ? "[NTP] sync OK" : "[NTP] sync FAILED");
    return ok;
}

void checkNtpResync(uint32_t now) {
    if ((int32_t)(now - nextNtpSyncMs) < 0) return;

    if (!wifiConnected) {
        // Wi-Fi未接続。ensureWifiConnected()の再接続を待ってから改めて試す。
        nextNtpSyncMs = now + NTP_RETRY_BASE_MS;
        return;
    }

    bool ok = syncNtpOnce();
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

    // ヒーターは自動制御しないため状態把握のみ。ここで読めなくても
    // checkMeterAndReport() 側で毎分リトライされるため、失敗しても先に進む。
    if (waitForPlugState("Heater Plug", PLUG_HEATER_MAC, heaterIsOn)) {
        logLine(String("[Heater Plug] OK state=") + (heaterIsOn ? "ON" : "OFF"));
    } else {
        logLine("[Heater Plug] NOT REACHABLE - will keep retrying");
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
#endif

    logLine("=== connectivity check done, starting automation ===");

    nextLightCheckMs = millis();
    nextSensorCheckMs = millis();
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

void checkLightSchedule(uint32_t now) {
    if ((int32_t)(now - nextLightCheckMs) < 0) return;

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 10)) {
        // まだ時刻が同期できていない。次のNTP同期を待つ(バックオフ対象外の固定間隔)。
        nextLightCheckMs = now + LIGHT_RETRY_BASE_MS;
        return;
    }

    bool wantOn = ControlLogic::computeDesiredLightOn(timeinfo, LIGHT_ON_HOUR, LIGHT_OFF_HOUR);
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

// =========================================================================
// 7. 温湿度計測・レポート送信
// =========================================================================

// 温湿度計をBLEスキャンし、取得できた値を都度FastAPIバックエンドに送信する
// (Wi-Fiは常時接続を前提とし、バッファは持たない)。同じタイミングでライト/ヒーターの
// 現在状態も確認し、前回報告値から変化していればあわせて送信する。
// スキャン/送信いずれかが失敗した場合は指数バックオフで再試行する。
void checkMeterAndReport(uint32_t now) {
    if ((int32_t)(now - nextSensorCheckMs) < 0) return;

    float tempC = 0.0f;
    uint8_t humidity = 0;
    bool ok = false;
    if (SwitchBotBLE::meterScanRead(METER_MAC, tempC, humidity)) {
        logLine("[Meter] " + String(tempC, 1) + "C " + String(humidity) + "%");

        // 温度がTEMP_MAX_Cを超えたらOFF、下回ったらONにする。既に同じ状態ならBLEコマンドは
        // 送らない(ライトのスケジュール制御と同じ考え方)。失敗した場合はheaterIsOnを更新
        // しないため、次回このループが回った時に改めて同じ切替を試みる。
        bool wantHeaterOn = ControlLogic::computeDesiredHeaterOn(tempC, TEMP_MAX_C);
        if (wantHeaterOn != heaterIsOn) {
            bool toggled = wantHeaterOn ? SwitchBotBLE::plugTurnOn(PLUG_HEATER_MAC) : SwitchBotBLE::plugTurnOff(PLUG_HEATER_MAC);
            if (toggled) {
                heaterIsOn = wantHeaterOn;
                logLine(String("[Heater] toggled -> ") + (heaterIsOn ? "ON" : "OFF"));
            } else {
                logLine("[Heater] toggle FAILED");
            }
        }

        bool haveLight = !lightReportKnown || uvbIsOn != lastReportedLightOn;
        bool haveHeater = !heaterReportKnown || heaterIsOn != lastReportedHeaterOn;

        if (wifiConnected) {
            ok = sendReadingHttp(tempC, humidity, haveLight, uvbIsOn, haveHeater, heaterIsOn);
            if (ok) {
                if (haveLight) {
                    lastReportedLightOn = uvbIsOn;
                    lightReportKnown = true;
                }
                if (haveHeater) {
                    lastReportedHeaterOn = heaterIsOn;
                    heaterReportKnown = true;
                }
            }
        } else {
            logLine("[Report] skipped (Wi-Fi not connected)");
        }
    } else {
        logLine("[Meter] scan failed");
    }

    if (ok) {
        meterFailureCount = 0;
        nextSensorCheckMs = now + TEMP_HUMIDITY_CHECK_INTERVAL_MS;
    } else {
        uint32_t delayMs = backoffDelayMs(METER_RETRY_BASE_MS, meterFailureCount, METER_RETRY_MAX_MS);
        meterFailureCount++;
        nextSensorCheckMs = now + delayMs;
        logLine("[Meter] retry in " + String(delayMs / 1000) + "s");
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
    turnDisplayOn(millis()); // 書き込み直後は5秒間だけ点灯する
    logLine("Reptile cage automation");

    // Wi-Fiに接続し、以後は常時接続を維持する(切断しない)。
    wifiConnected = connectWifiWithRetry();
    if (wifiConnected) {
        logLine("[WiFi] connected");
        timeSynced = syncNtpOnce();
    } else {
        logLine("[WiFi] NOT CONNECTED - will keep retrying");
    }

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

    ensureWifiConnected(now);
    checkNtpResync(now);
    checkLightSchedule(now);
    checkMeterAndReport(now);
    checkDisplayTimeout(now);
    // checkTempHumidity(now); // ヒーター・ミストの判定。ライトのみ制御への切替に伴い無効化 (上記#if 0参照)

    if (M5.BtnA.wasPressed()) {
        turnDisplayOn(now);
        showCurrentTime();
    }

    delay(10);
}
