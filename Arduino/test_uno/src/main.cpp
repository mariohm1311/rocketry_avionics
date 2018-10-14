#include <Arduino.h>

uint16_t iter = 0;
const uint8_t ledPin = 13;

void setup() {
    Serial.begin(9600);
    pinMode(ledPin, OUTPUT);
}

void loop() {
    digitalWrite(ledPin, HIGH);
    Serial.println(iter);
    iter++;
    delay(250);
    digitalWrite(ledPin, LOW);
    delay(250);
}