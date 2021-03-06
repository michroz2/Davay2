/*
  LoRa RX, based on Duplex communication wth callback example

  Listens for a message from TX, about the button pressed/released.
  Turnes LED ON/OFF and replies to TX when receives.
  Also replies in "Pairing mode"
  Для другого диапазона надо пересмотреть некоторые дефайны
  Для каждого RX выбрать MY_ADDRESS

*/
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <EEPROM.h>

#ifdef ARDUINO_SAMD_MKRWAN1300  //not my code :)
#error "This code is not compatible with the Arduino MKR WAN 1300 board!"
#endif

// Дебагирование: раскомментить для использования 1 строчку:
#define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.print(String(millis())+" "+x)
#define DEBUGln(x) Serial.println(String(millis())+" "+x)
#else
#define DEBUG(x)
#define DEBUGln(x)
#endif

//==== MILLIS TIMER MACRO ====
// performs the {subsequent} code once and then, if needed, again after each x ms
#define EVERY_MS(x) \
  static uint32_t tmr;\
  bool flg = millis() - tmr >= (x);\
  if (flg) tmr = millis();\
  if (flg)
//===========================

#define CALL_FQ 434E6                       //стартовая рабочая частота.
#define MIN_FQ 433.05E6                     //минимальная частота диапазона.
#define MAX_FQ 434.79E6                     //максимальная рабочая частота.
#define CHANNEL_WIDTH 1E5                 // поставить 100 или 200 KHz
#define NUM_LORA_CHANNELS (MAX_FQ - MIN_FQ)/CHANNEL_WIDTH   //количество каналов столько кГц
long minFQ = round(MIN_FQ / 1.0E5) * 1.0E5;
//TODO: сделать выбор рабочего канала:
#define WORKING_CHANNEL 5                 //this supposed to be defined by jumpers or by scan
#define BROADCAST_ADDRESS 0xFF
#define DEFAULT_TIMEOUT 300               //начальный таймаут для получения фидбяка (мс)
#define PAIRING_TIMEOUT 1000  //ms
#define PAIRING_START_ATTEMPTS  2
#define PAIRING_COMM_ATTEMPTS 5
#define WORK_COMM_ATTEMPTS 3
#define PING_TIMEOUT 5000  //ms
#define PING_FLASH_ACTIVE 200  //ms
#define PING_FLASH_PAUSE 400  //ms

#define PIN_LED  5  // Номер пина Arduino, к которому подключен вывод статусного ЛЕД-а 
#define PIN_SIGNAL  6  // Номер пина Arduino, к которому подключен вывод сигнала
#define PIN_PAIR  4  // Номер пина Arduino, к которому подключен вывод кнопки Паринга (притянуто к 5в)

//Просто чтобы не забыть, что такое возможно для SPI, вместо дефолтов:
//const int csPin = 10;          // LoRa radio chip select
//const int resetPin = 9;       // LoRa radio reset
//const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

//Практически, описание протокола:
#define CMD_PAIRING     200 //TX передаёт команду/команду с бродкастным адресом
#define CMD_PAIRING_OK  201 //RX отвечает /свой зашитый адрес с бродкастным адресом
#define CMD_ADDR        202 //TX передаёт по адресу полученный адрес/адрес
#define CMD_ADDR_OK     203 //RX отвечает /адрес с адресом
#define CMD_CHANNEL     204 //TX передаёт свой будущий /канал
#define CMD_CHANNEL_OK  205 //RX отвечает, что переключается/канал
#define CMD_START       206 //TX передаёт на канале старт/старт
#define CMD_START_OK    207 //RX отвечает на канале ok/ok - паринг закончился
#define CMD_LED         208 //TX передаёт сигнал на изменение состояния лед-а
#define CMD_LED_OK      209 //RX подтверждает
#define CMD_PING        212 //TX пингует периодически с /состоянием леда
#define CMD_PONG        213 //RX отвечает с состоянием леда
#define CMD_PING_OK     213 //то же что предыдущее
//Specific for each RX:
#define MY_ADDRESS      78

byte workAddress = MY_ADDRESS;  // address of connection
byte msgNumber = 0;                    // = number of the received message: reply always this number
byte rcvCmd = CMD_PONG;          // expected command (Default = PONG)
byte rcvData;                         // additional data byte received
byte sndCmd = CMD_PONG;              // outgoing command Default = PING
byte sndData;                         // additional data byte sent
bool signalStatus;                     //last received TX Button status
bool statePairing;                    //state of pairing: 1 or 0 (no pairing)
byte workChannel;                      //channel to send to RX while paring and work on it
unsigned long workFrequency = CALL_FQ; //working Frequency

long lastSendTime = 0;                // last send time
int lastRSSI;
float lastSNR;
unsigned long lastTurnaround;         // round-trip time between tx and rx
long lastFrequencyError;

//bool currButtonState;               //Current state of Button
//bool prevButtonState;               //Previous state of Button
bool currPairingButtonState;              //State of Pairing button/pin
bool prevPairingButtonState;              //State of Pairing button/pin

unsigned long pingFlashPhase;
unsigned long pingFlashTime;
bool pingFlash;

void setup() {//=======================SETUP===============================

  // initialize serial
#ifdef DEBUG_ENABLE
  Serial.begin(9600);
  while (!Serial);
#endif

  DEBUGln("================================");
  DEBUGln("=========== START RX ===========");
  DEBUGln("Davay LoRa RX setup()");

  // override the default CS, reset, and IRQ pins (optional)
  //  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  //INIT PINS
  pinMode(PIN_SIGNAL, OUTPUT);
  pinMode(PIN_PAIR, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  workFrequency = CALL_FQ;
  if (!LoRa.begin(workFrequency)) {             // initialize radio at CALL Frequency
    DEBUGln("LoRa init failed. Check your connections.");
    while (true) {
      flashlLedError();    // if failed, do nothing
    }
  }

  setLoRaParams();

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa.receive(); //Always listen by default

  DEBUGln("LoRa RX init success.");

  if (readEEPROM()) {
    //    setCommData();
    statePairing = false;
  }
  else {    //Ecли не было ничего записано в EEPROM, то начать Паринг
    statePairing = true;
  }
  DEBUGln("Setup complete, Pairing: " + String(statePairing));
}//setup      //=======================SETUP===============================

void loop() { //  ===!!!===!!!===!!!===!!!===!!!===!!!===!!!===!!!===!!!===

  processPairingButton();
  processPing();  //see if we haven't lost connection to TX
  processSignal();

}//loop()         ===!!!===!!!===!!!===!!!===!!!===!!!===!!!===!!!===!!!===

void   processPairingButton() {
  if (!statePairing) {
    currPairingButtonState = !digitalRead(PIN_PAIR); // Читаем состояние кнопки 1=нажата; 0=отпущена
    if ((prevPairingButtonState != currPairingButtonState) && (currPairingButtonState == 1)) {
      //the previously released button becomes pressed - we do pairing.
      prevPairingButtonState = currPairingButtonState;
      statePairing = true;
      signalStatus = false;
      DEBUGln("Pairing Button worked!");
      flashLedParing();
    }
  }
}// processPairingButton()

void   processPing() {
  if ((millis() - lastSendTime) > PING_TIMEOUT) { // if long time no signal from TX
    signalStatus = false;
    if ((millis() - pingFlashTime) > pingFlashPhase) {
      pingFlash = !pingFlash;
      if (pingFlash) {
        pingFlashPhase = PING_FLASH_ACTIVE;
      } else {
        pingFlashPhase = PING_FLASH_PAUSE;
      }
      pingFlashTime = millis();
      updateLed(pingFlash);
    }
  }
}// processPing()

void  processSignal() {
  digitalWrite(PIN_SIGNAL, signalStatus);
}


void updateLed(bool ledStatus) { // turn ON or OFF the status LED
  digitalWrite(PIN_LED, ledStatus);
  DEBUGln("update Led(): " + String(ledStatus));
}// updateLed(bool ledStatus)

void flashlLedError() { //flash 3 times total 1.5 sec
  DEBUGln("flashlLedError()");
  bool flash = false;
  for (int i = 0; i < 6; i++) {
    flash = !flash;
    updateLed(flash);
    delay(200);
  }
  delay(300);
}

void flashLedParing() { //flash 2 times before each pairing communication - total 1.5 sec
  DEBUGln("flashLedParing()");
  bool flash = false;
  for (int i = 0; i < 4; i++) {
    flash = !flash;
    updateLed(flash);
    delay(200);
  }
  delay(300);
}

void flashLedParingOK() { //flash 1 long time - total 1.5 sec
  DEBUGln("flashLedParingOK()");
  bool flash = false;
  for (int i = 0; i < 2; i++) {
    flash = !flash;
    updateLed(flash);
    delay(600);
  }
  delay(300);
}

long frequencyByChannel(byte numChannel) {
  DEBUGln("frequencyByChannel(byte numChannel)");
  if (numChannel > NUM_LORA_CHANNELS) {
    DEBUGln("Invalid Channel: " + String(numChannel));
    return CALL_FQ;
  }
  return (minFQ + numChannel * CHANNEL_WIDTH);
}
// done frequencyByChannel(byte numChannel)

bool readEEPROM() {
  DEBUGln("readEEPROM()");
  return false;
} //TODO: readEEPROM()

void writeEEPROM() { //Store Channel and Address in EEPROM for next time
  DEBUGln("writeEEPROM()");
  DEBUGln("WorkAddress: " + String(workAddress));
  DEBUGln("WorkChannel: " + String(workChannel));
  DEBUGln("workFrequency: " + String(workFrequency));
} //TODO: writeEEPROM()

void setLoRaParams() {
  DEBUGln("setLoRaParams()");
  //Set LoRa for Longest Range:
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);    //max
  LoRa.setSignalBandwidth(125E3);                 //..31.25E3, 41.7E3, 62.5E3, (125E3), and 250E3.
  LoRa.setSpreadingFactor(8);                    //default = 7
  LoRa.setPreambleLength(6);                    //min = 6, default = 8
  LoRa.enableCrc();                             //
  //  LoRa.setCodingRate4(5);

}// DONE void setLoRaParams()

void sendMessage(byte msgAddr, byte msgNumber, byte msgCmd, byte msgData) {
  //  DEBUGln("sendMessage(byte msgCmd, byte sndData)");
  while (!LoRa.beginPacket()) {
    DEBUGln("Waiting to begin REPLY");
  }                   // start packet
  LoRa.write(msgAddr);              // reply address
  LoRa.write(msgNumber);              // reply Msg Number
  LoRa.write(msgCmd);                  // reply command
  LoRa.write(msgData);                 // reply Data
  while (!LoRa.endPacket()) {            // finish packet and send it
    DEBUGln("Waiting to finish REPLY");
  }
  lastSendTime = millis();            // timestamp the message
  LoRa.receive();                     // go back into receive mode
  DEBUGln(("sendMessage done! ") + String(workAddress)\
          + " " + String(msgNumber) + " " + String(msgCmd) + " " + String(msgData));
}// void sendMessage(byte messageByte)

void onTxDone() {
  DEBUGln("onTxDone()");
  lastSendTime = millis();            // timestamp the message
  LoRa.receive();                     // go back into receive mode
}

void onReceive(int packetSize) {
  DEBUGln("Package Received!");
  if (packetSize != 4) {
    DEBUGln("Wrong Packet Size: " + String(packetSize));
    return;          // not our packet, return
  }
  // read packet header bytes:
  byte rcvAddress = LoRa.read();          // received address
  byte rcvCount = LoRa.read();            // received Number
  byte rcvCmd = LoRa.read();              // received command
  byte rcvData = LoRa.read();                  // received data

  lastRSSI =  LoRa.packetRssi();
  lastSNR = LoRa.packetSnr();
  lastTurnaround = millis() - lastSendTime;
  lastFrequencyError = LoRa.packetFrequencyError();

  if (!((rcvAddress == workAddress) || ((rcvAddress == BROADCAST_ADDRESS) && (statePairing) && (rcvCmd == CMD_PAIRING)))) {
    DEBUGln("Wrong Address: " + String(rcvAddress) + " - " +  String(workAddress));
    return;
  }

  msgNumber = rcvCount;

  switch (rcvCmd) {
    case CMD_PAIRING:
    case CMD_ADDR:
    case CMD_CHANNEL:
    case CMD_START:
      if (!statePairing) {
        DEBUGln("Invalid command, when NOT Pairing: " + String(rcvCmd));
        return;
      }
      break;
    case CMD_LED:
    case CMD_PING:
      if (statePairing) {
        DEBUGln("Invalid command, when Pairing: " + String(rcvCmd));
        return;
      }
      break;
    default:
      DEBUGln("Unknown command: " + String(rcvCmd));
  }

  DEBUGln("RSSI: " + String(lastRSSI));
  DEBUGln("Snr: " + String(lastSNR));
  DEBUGln("Turnaround: " + String(lastTurnaround));
  DEBUGln("Frequency Error: " + String(lastFrequencyError));
  DEBUGln("Received Message: "  + String(rcvAddress) + " " + String(rcvCount)\
          +" " + String( rcvCmd) + " " + String( rcvData));

  switch (rcvCmd) {
    case CMD_PAIRING:
      DEBUGln("===CMD_PAIRING===");
      workAddress = MY_ADDRESS;
      sendMessage(rcvAddress, msgNumber, CMD_PAIRING_OK, workAddress);
      break;
    case CMD_ADDR:
      DEBUGln("===CMD_ADDR===");
      workAddress = rcvData;
      sendMessage(rcvAddress, msgNumber, CMD_ADDR_OK, workAddress);
      break;
    case CMD_CHANNEL:
      DEBUGln("===CMD_CHANNEL===");
      workChannel = rcvData;
      sendMessage(rcvAddress, msgNumber, CMD_CHANNEL_OK, workChannel);
      workFrequency = frequencyByChannel(workChannel);
//      LoRa.setFrequency(workFrequency);
      DEBUGln("setFrequency: " + String(workFrequency));
      break;
    case CMD_START:
      DEBUGln("===CMD_START===");
      sendMessage(rcvAddress, msgNumber, CMD_START_OK, CMD_START_OK);
      statePairing = false;
      break;
    case CMD_LED:
      DEBUGln("===CMD_LED===");
      signalStatus = rcvData;
      sendMessage(rcvAddress, msgNumber, CMD_LED_OK, signalStatus);
      break;
    case CMD_PING:
      DEBUGln("===CMD_PING===");
      signalStatus = rcvData;
      sendMessage(rcvAddress, msgNumber, CMD_PONG, signalStatus);

  }

}//void onReceive(int packetSize)
