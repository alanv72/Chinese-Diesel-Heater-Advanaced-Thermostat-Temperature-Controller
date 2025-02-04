#include "ESP32SoftwareSerial.h"
#include <Arduino.h>

ESP32SoftwareSerial::ESP32SoftwareSerial(uint8_t p) : pin(p) {
    //pinMode(pin, INPUT_PULLUP);
    pinMode(pin, INPUT);
    bitTime = 1000000 / 25000; // 25000 baud = 40us per bit
}

void ESP32SoftwareSerial::begin(int speed) {
    bitTime = 1000000 / speed;
}

void ESP32SoftwareSerial::write(uint8_t byte) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW); delayMicroseconds(bitTime); // Start bit
    for (int i = 0; i < 8; i++) {
        digitalWrite(pin, (byte & 1) ? HIGH : LOW);
        delayMicroseconds(bitTime);
        byte >>= 1;
    }
    digitalWrite(pin, HIGH); delayMicroseconds(bitTime); // Stop bit
    //pinMode(pin, INPUT_PULLUP);
    pinMode(pin, INPUT);
}

int ESP32SoftwareSerial::read() {
    while (digitalRead(pin) == HIGH); // Wait for start bit
    delayMicroseconds(bitTime / 2); // Center of start bit
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
        delayMicroseconds(bitTime);
        result >>= 1;
        if (digitalRead(pin) == HIGH) result |= 0x80;
    }
    delayMicroseconds(bitTime); // Skip stop bit
    return result;
}

bool ESP32SoftwareSerial::available() {
    return digitalRead(pin) == LOW; // Assuming low means start of new data
}