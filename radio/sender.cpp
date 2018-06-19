//https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}
void loop() {
  const char longitude[] = "hello";
  const char lat[] = "world";
  const char longitude[];
  const char lat[];
  longitude[] = Serial.readln();
  lat[] = Serial.readln();
  radio.write(&longitude, sizeof(longitude));
  radio.write(&lat, sizeof(lat));
  delay(1000);
}
