#include "control_logic.h"

#include <cstdio>

namespace ControlLogic {

bool computeDesiredLightOn(const struct tm& t, int lightOnHour, int lightOffHour) {
    return t.tm_hour >= lightOnHour && t.tm_hour < lightOffHour;
}

bool computeDesiredHeaterOn(float tempC, float tempMaxC) {
    return tempC < tempMaxC;
}

std::string buildReadingRequestBody(
    float tempC,
    uint8_t humidity,
    bool haveLight,
    bool lightOn,
    bool haveHeater,
    bool heaterOn
) {
    char tempBuf[16];
    std::snprintf(tempBuf, sizeof(tempBuf), "%.1f", static_cast<double>(tempC));

    std::string body = std::string("{\"temp_c\":") + tempBuf + ",\"humidity\":" + std::to_string(humidity);
    if (haveLight) body += std::string(",\"is_light_on\":") + (lightOn ? "true" : "false");
    if (haveHeater) body += std::string(",\"is_heater_on\":") + (heaterOn ? "true" : "false");
    body += "}";
    return body;
}

} // namespace ControlLogic
