#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
void loop() {
  if (radio.available()) {
    int longitude;
    int lat;
    radio.read(&longitude, sizeof(longitude));
    Serial.println(longitude);
    radio.read(&lat, sizeof(lat));
    Serial.println(lat);
  }
}


#include <SPI.h>
#include <stdlib.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
void loop() {
  if (radio.available()) {
    Serial.println("radio available (:");
    const char message[6];
    radio.read(&message, sizeof(message));
    Serial.println(message);
    delay(1000);
  }
  else{
    Serial.println("radio not available ):");
  }
}
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
 const char message[6]="hello";
 radio.write(message, sizeof(message));
  delay(1000);
}
