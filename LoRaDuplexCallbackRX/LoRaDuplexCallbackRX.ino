/*
  LoRa RX, based on Duplex communication wth callback

  Listens for a message from TX, about the button pressed/released.
  Turnes LED ON/OFF and replies to TX when receives.
  
*/
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <EEPROM.h>

//#define PIN_BUTTON  6  // Номер пина Arduino, к которому подключен вывод кнопки
#define PIN_LED  5  // Номер пина Arduino, к которому подключен вывод сигнальный LED

#define RECEIVE_FQ 433.12E6 //рабочая частота 
// SEND_FQ 433.5E6 //частота посылки, если надо - должна совпадать с рабочей частотой
//передатчика и наоборот. Если не нужна - откомментировать!

#ifdef ARDUINO_SAMD_MKRWAN1300
#error "This example is not compatible with the Arduino MKR WAN 1300 board!"
#endif

//const int csPin = 7;          // LoRa radio chip select
//const int resetPin = 6;       // LoRa radio reset
//const int irqPin = 1;         // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message
byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBA;     // address of this device
byte destination = 0xAB;      // destination to send to
long lastSendTime = 0;        // last send time
long reactionTime = 0;        // time between rx and tx
int interval = 100;          // interval between sends

//bool currState;               //Current state of Button
//bool prevState;               //Previous state of Button
//bool stateChanged;            //Button pressed or released
bool ledStatus;                //status of the  led

String turnON = "ON";
String turnOFF = "OFF";

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  Serial.println("LoRa RX Duplex with callback");

  // override the default CS, reset, and IRQ pins (optional)
  //  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(RECEIVE_FQ)) {             // initialize radio at 433 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  //Set LoRa for Longest Range:
  LoRa.setTxPower(20);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setSpreadingFactor(7);
  //  LoRa.setCodingRate4(5);

  LoRa.onReceive(onReceive);
#ifdef SEND_FQ
  LoRa.setFrequency(RECEIVE_FQ);
#endif
  LoRa.receive();

  //INIT PINS
  //  pinMode(PIN_BUTTON,INPUT);
  pinMode(PIN_LED, OUTPUT);

  Serial.println("LoRa RX init succeeded.");
}

void loop() {

  //  currState = digitalRead(PIN_BUTTON); // Читаем состояние кнопки 1=нажата; 0=отпущена
  //  stateChanged = (currState != prevState);

  if (millis() - lastSendTime > interval) {  // || stateChanged ) {

    //    ledStatus = !ledStatus; //TEST MODE!!!

    String message = commandMessage(ledStatus); //"Ping from RX";   // send a message
    sendMessage(message);
    Serial.println("Pinging " + message);
    interval = random(2000) + 2000;     // 2-4 seconds
  }

}

void sendMessage(String outgoing) {
#ifdef SEND_FQ
  LoRa.setFrequency(SEND_FQ);
#endif
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  //  LoRa.write(localAddress);             // add sender address
  //  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it

  lastSendTime = millis();            // timestamp the message

  msgCount++;                           // increment message ID
#ifdef SEND_FQ
  LoRa.setFrequency(RECEIVE_FQ);
#endif
  LoRa.receive();                     // go back into receive mode


}

String commandMessage(bool state) {
  if (state) {
    return ("ON");
  }
  return ("OFF");
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  // read packet header bytes:
  reactionTime = millis();
  int recipient = LoRa.read();          // recipient address
  //  byte sender = LoRa.read();            // sender address
  //  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";                 // payload of packet

  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add bytes one by one
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  //  Serial.println("Received from: 0x" + String(sender, HEX));
  //  Serial.println("Sent to: 0x" + String(recipient, HEX));
  //  Serial.println("Message ID: " + String(incomingMsgId));
  //  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println("MS: " + String(millis() - lastSendTime));

  if (incoming == turnON) {
    //Включить
    ledStatus = 1;
  } else if (incoming == turnOFF) {
    //Выключить
    ledStatus = 0;
  } else {
    Serial.println("Invalid Command");
  }
  digitalWrite(PIN_LED, ledStatus);
  sendMessage(commandMessage(ledStatus));
  //  Serial.println("RT: " + String(millis() - reactionTime));
  Serial.println();

}
