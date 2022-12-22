#include <Arduino.h>
#include <rpm_controller.hpp>
#define TACH_PIN 32
#define PWM_PIN 33
#define MAX_RPM 1700
static rpm_controller fan(TACH_PIN,PWM_PIN,MAX_RPM);
void setup() {
    Serial.begin(115200);
    
    fan.initialize();
    fan.rpm(fan.max_rpm());
}
static uint32_t ts = 0;
void loop() {
    uint32_t ms = millis();
    if(ts+500<=ms) {
        ts = ms;
        Serial.print(fan.rpm());
        Serial.print("/");
        Serial.println(fan.max_rpm());
    }
}