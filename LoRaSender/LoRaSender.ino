#include <SPI.h>
#include <LoRa.h>

#define PIN_BUTTON  6  // Номер пина Arduino, к которому подключен вывод кнопки
#define PIN_FB_LED  5  // Номер пина Arduino, к которому подключен вывод LED непосредственной обратной связи

int counter = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender");

  while (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    delay(500);
//    while (1);

//INIT PINS для ЛЭДа обратной связи и главной кнопки
  pinMode(PIN_BUTTON,INPUT);
  pinMode(PIN_FB_LED,OUTPUT);

  }
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;

  delay(2000);
}
