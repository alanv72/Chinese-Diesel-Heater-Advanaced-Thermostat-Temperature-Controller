#ifndef ESP32SoftwareSerial_h
#define ESP32SoftwareSerial_h

#include <Arduino.h>  // This includes uint8_t and other Arduino functions

class ESP32SoftwareSerial {
private:
    uint8_t pin;
    unsigned long bitTime;

public:
    ESP32SoftwareSerial(uint8_t p);
    void begin(int speed);
    void write(uint8_t byte);
    int read();
    bool available();
    // void stopListening();
    // void startListening();
};

#endif