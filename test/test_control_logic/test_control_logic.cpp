// src/common/control_logic.cpp の単体テスト。BLE/WiFiを一切使わない純粋なロジックだけを
// 対象にする(ハードウェア依存部分はここではテストできない)。
// 実行: pio test -e native

#include <unity.h>

#include "control_logic.h"

namespace {

struct tm makeTime(int hour) {
    struct tm t = {};
    t.tm_hour = hour;
    return t;
}

} // namespace

void setUp(void) {}
void tearDown(void) {}

// ---- computeDesiredLightOn: 7:00-18:59 ON / 19:00-6:59 OFF ----

void test_light_on_during_day(void) {
    TEST_ASSERT_TRUE(ControlLogic::computeDesiredLightOn(makeTime(12), 7, 19));
}

void test_light_off_before_on_hour(void) {
    TEST_ASSERT_FALSE(ControlLogic::computeDesiredLightOn(makeTime(6), 7, 19));
}

void test_light_on_at_on_hour_boundary(void) {
    TEST_ASSERT_TRUE(ControlLogic::computeDesiredLightOn(makeTime(7), 7, 19));
}

void test_light_off_at_off_hour_boundary(void) {
    TEST_ASSERT_FALSE(ControlLogic::computeDesiredLightOn(makeTime(19), 7, 19));
}

void test_light_off_late_night(void) {
    TEST_ASSERT_FALSE(ControlLogic::computeDesiredLightOn(makeTime(23), 7, 19));
}

// ---- computeDesiredHeaterOn: 温度 < 上限ならON ----

void test_heater_on_below_threshold(void) {
    TEST_ASSERT_TRUE(ControlLogic::computeDesiredHeaterOn(29.9f, 30.0f));
}

void test_heater_off_at_threshold(void) {
    TEST_ASSERT_FALSE(ControlLogic::computeDesiredHeaterOn(30.0f, 30.0f));
}

void test_heater_off_above_threshold(void) {
    TEST_ASSERT_FALSE(ControlLogic::computeDesiredHeaterOn(30.1f, 30.0f));
}

// ---- buildReadingRequestBody: 変化した項目だけをJSONに含める ----

void test_body_includes_only_temp_and_humidity_by_default(void) {
    std::string body = ControlLogic::buildReadingRequestBody(27.3f, 55, false, false, false, false);
    TEST_ASSERT_EQUAL_STRING("{\"temp_c\":27.3,\"humidity\":55}", body.c_str());
}

void test_body_includes_light_when_changed(void) {
    std::string body = ControlLogic::buildReadingRequestBody(27.3f, 55, true, true, false, false);
    TEST_ASSERT_EQUAL_STRING("{\"temp_c\":27.3,\"humidity\":55,\"is_light_on\":true}", body.c_str());
}

void test_body_includes_heater_when_changed(void) {
    std::string body = ControlLogic::buildReadingRequestBody(27.3f, 55, false, false, true, false);
    TEST_ASSERT_EQUAL_STRING("{\"temp_c\":27.3,\"humidity\":55,\"is_heater_on\":false}", body.c_str());
}

void test_body_includes_both_when_both_changed(void) {
    std::string body = ControlLogic::buildReadingRequestBody(27.3f, 55, true, false, true, true);
    TEST_ASSERT_EQUAL_STRING(
        "{\"temp_c\":27.3,\"humidity\":55,\"is_light_on\":false,\"is_heater_on\":true}", body.c_str()
    );
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_light_on_during_day);
    RUN_TEST(test_light_off_before_on_hour);
    RUN_TEST(test_light_on_at_on_hour_boundary);
    RUN_TEST(test_light_off_at_off_hour_boundary);
    RUN_TEST(test_light_off_late_night);
    RUN_TEST(test_heater_on_below_threshold);
    RUN_TEST(test_heater_off_at_threshold);
    RUN_TEST(test_heater_off_above_threshold);
    RUN_TEST(test_body_includes_only_temp_and_humidity_by_default);
    RUN_TEST(test_body_includes_light_when_changed);
    RUN_TEST(test_body_includes_heater_when_changed);
    RUN_TEST(test_body_includes_both_when_both_changed);
    return UNITY_END();
}
