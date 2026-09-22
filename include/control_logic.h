#pragma once

// ライト/ヒーターの判定・レポート送信ペイロード組み立てのうち、ハードウェア(BLE/WiFi)に
// 依存しない純粋なロジックだけを切り出したもの。ネイティブ環境(pio test -e native)で
// 単体テストできるようにするための分離であり、Arduino/ESP32固有の型には依存しない。

#include <cstdint>
#include <ctime>
#include <string>

namespace ControlLogic {

// 実時刻(t.tm_hour)がlightOnHour以上lightOffHour未満ならライトはONにすべきと判定する。
bool computeDesiredLightOn(const struct tm& t, int lightOnHour, int lightOffHour);

// 温度がtempMaxC未満ならヒーターはONにすべきと判定する(ヒステリシス無しの単純な閾値制御)。
bool computeDesiredHeaterOn(float tempC, float tempMaxC);

// POST /api/readings 用のJSONボディを組み立てる。haveLight/haveHeaterがfalseの場合は
// 対応するフィールドをJSONに含めない(バックエンドはそれをNULLのまま保持する)。
std::string buildReadingRequestBody(
    float tempC,
    uint8_t humidity,
    bool haveLight,
    bool lightOn,
    bool haveHeater,
    bool heaterOn
);

} // namespace ControlLogic
