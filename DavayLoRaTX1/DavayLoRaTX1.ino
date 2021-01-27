/*
  LoRa Davay TX, based on Duplex communication with callback example
  Sends a message when button is pressed/released or pings on timer
  Waits for reply on callback
  Also makes "Pairing" with RX device.

  Для другого диапазона надо пересмотреть некоторые дефайны

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
#define PING_TIMEOUT 3000  //ms
#define PING_FLASH 100  //ms

#define PIN_BUTTON  6  // Номер пина Arduino, к которому подключен вывод кнопки (притянуто к 5в)
#define PIN_LED  5  // Номер пина Arduino, к которому подключен вывод LED обратной связи
#define PIN_PAIR  4  // Номер пина Arduino, к которому подключен вывод кнопки Паринга (притянуто к 5в)

//Просто чтобы не забыть, что такое возможно для SPI, вместо дефолтов:
//const int csPin = 10;          // LoRa radio chip select
//const int resetPin = 9;       // LoRa radio reset
//const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

//Практически, описание протокола:
#define CMD_PAIRING     200 //TX передаёт команду/команду с бродкастным адресом
#define CMD_PAIRING_OK  201 //RX отвечает /свой зашитый адрес с бродкастным адресом
#define CMD_ADDR        202 //TX передаёт полученный адрес/адрес с адресом :)
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

byte workAddress = BROADCAST_ADDRESS;  // address of connection
byte msgCount = 0;                    // = number of outgoing message
byte sndCmd = CMD_PING;              // outgoing command Default = PING
byte cmdExpected = CMD_PONG;          // expected command (Default = PONG)
byte rcvData;                         // additional data byte received
byte sndData;                         // additional data byte sent
bool wasReceived;                     //indication of reply message received
byte workChannel;                      //channel to send to RX while paring and work on it
unsigned long workFrequency = CALL_FQ; //working Frequency

long lastSendTime = 0;                // last send time
int lastRSSI;
float lastSNR;
unsigned long lastTurnaround;         // round-trip time between tx and rx
long lastFrequencyError;
unsigned long expectedTimeout = DEFAULT_TIMEOUT;

bool currButtonState;               //Current state of Button
bool prevButtonState;               //Previous state of Button
bool currPairingButtonState;              //State of Pairing button/pin
bool prevPairingButtonState;              //State of Pairing button/pin

unsigned long pingTimer;
unsigned long pingFlashTimer;
bool pingFlash;

void setup() {//=======================SETUP===============================

  // initialize serial
#ifdef DEBUG_ENABLE
  Serial.begin(9600);
  while (!Serial);
#endif

  DEBUGln("================================");
  DEBUGln("=========== START TX ===========");
  DEBUGln("Davay LoRa TX setup()");

  // override the default CS, reset, and IRQ pins (optional)
  //  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  //INIT PINS
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_PAIR, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  if (!LoRa.begin(CALL_FQ)) {             // initialize radio at CALL Frequency
    DEBUGln("LoRa init failed. Check your connections.");
    while (true) {
      flashlLedError();    // if failed, do nothing
    }
  }

  setLoRaParams();

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa.idle(); //Until we decide how to continue

  DEBUGln("LoRa TX init success.");

  if (readEEPROM()) {
    //    setCommData();
  }
  else {    //Ecли не было ничего записано в EEPROM, то начать Паринг
    while (!pairing()) {
      DEBUGln("Initial Pairing unsuccessful");
    }
  }

}//setup      //======================= SETUP ===============================

void loop() { //  ===!!!===!!!===!!!===!!!= LOOP =!!!===!!!===!!!===!!!===!!!===
  processPairingButton();
  processButton();
  processPing();

}//loop()         ===!!!===!!!===!!!===!!!= LOOP =!!!===!!!===!!!===!!!===!!!===

void   processPairingButton() {
  currPairingButtonState = !digitalRead(PIN_PAIR); // Читаем состояние кнопки 1=нажата; 0=отпущена
  if ((prevPairingButtonState != currPairingButtonState) && (currPairingButtonState == 1)) {
    //the previously released button becomes pressed - we do pairing.
    DEBUGln("processPairingButton()");
    prevPairingButtonState = currPairingButtonState;
    while (!pairing()) {
      DEBUGln("Forced Pairing unsuccessful");
    }
  }
}

void   processButton() {
  currButtonState = !digitalRead(PIN_BUTTON); // Читаем состояние кнопки 1=нажата; 0=отпущена
  if (prevButtonState != currButtonState) {   //button pressed or released
    DEBUGln("processButton()");
    prevButtonState = currButtonState;
    if (commSession( CMD_LED, currButtonState, CMD_LED_OK, 2 * lastTurnaround, WORK_COMM_ATTEMPTS )) {
      updateFBLed(currButtonState);
    }
    else {
      updateFBLed(false);
      flashlLedError();
    }
  }
}

void   processPing() {
  if (pingFlash) {// ping reply already received, processing led flash only
    if ((millis() - pingFlashTimer) > PING_FLASH) { //end led flash
      pingFlash = false;
      updateFBLed(false);
    }
    return;
  }
  if ((millis() - pingTimer) > PING_TIMEOUT) { // long time no comm
    DEBUGln("processPing()");
    if (commSession( CMD_PING, currButtonState, CMD_PONG, 2 * lastTurnaround, WORK_COMM_ATTEMPTS )) {
      updateFBLed(true);
      pingFlash = true;
      pingFlashTimer = millis();
    }
    else {
      updateFBLed(false);
      flashlLedError();
    }
  }
}

void updateFBLed(bool ledStatus) { // turn ON or OFF the Feedback LED
  digitalWrite(PIN_LED, ledStatus);
  //  DEBUGln("updateFBLed(): " + String(ledStatus));
}

void flashlLedError() { //flash 3 times total 1.5 sec
  DEBUGln("flashlLedError()");
  bool flash = false;
  for (int i = 0; i < 6; i++) {
    flash = !flash;
    updateFBLed(flash);
    delay(200);
  }
  delay(300);
}

void flashLedParing() { //flash 2 times before each pairing communication - total 1.5 sec
  DEBUGln("flashLedParing()");
  bool flash = false;
  for (int i = 0; i < 4; i++) {
    flash = !flash;
    updateFBLed(flash);
    delay(200);
  }
  delay(300);
}

void flashLedParingOK() { //flash 1 long time - total 1.5 sec
  DEBUGln("flashLedParingOK()");
  bool flash = false;
  for (int i = 0; i < 2; i++) {
    flash = !flash;
    updateFBLed(flash);
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

byte workingChannel() {
  DEBUGln("workingChannel()");
  //TODO: define the best and unused channel
  return WORKING_CHANNEL; //so far hardcoded;
}

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

void sendMessage(byte msgCmd, byte sndData) {
  //  DEBUGln("sendMessage(byte msgCmd, byte sndData)");
  while (!LoRa.beginPacket()) {
    DEBUGln("Waiting to begin TX");
  }                   // start packet
  LoRa.write(workAddress);              // add address
  LoRa.write(++msgCount);              // add Msg Number
  LoRa.write(msgCmd);                  // add command
  LoRa.write(sndData);                 // add Data
  while (!LoRa.endPacket()) {            // finish packet and send it
    DEBUGln("Waiting to finish TX");
  }
  lastSendTime = millis();            // timestamp the message
  LoRa.receive();                     // go back into receive mode
  DEBUGln("sendMessage done! " + String(workAddress)\
          + " " + String(msgCount) + " " + String(msgCmd) + " " + String(sndData));
}// void sendMessage(byte messageByte)

void onTxDone() {
  DEBUGln("onTxDone()");
  lastSendTime = millis();            // timestamp the message
  LoRa.receive();                     // go back into receive mode
}

void onReceive(int packetSize) {
  DEBUGln("onReceive(int packetSize)");
  if (packetSize != 4) {
    DEBUGln("Invalid Packet Size: " + String(packetSize));
    return;          // not our packet, return
  }
  // read packet header bytes:
  byte rcvAddress = LoRa.read();          // replied address
  if (rcvAddress != workAddress) {
    DEBUGln("Invalid Address: " + String(rcvAddress) + ", Expected: " + String(workAddress));
    return;
  }
  byte rcvCount = LoRa.read();    // replied number of Message
  if (rcvCount != msgCount) {
    DEBUGln("Invalid Number: " + String(rcvCount) + ", Expected: " + String(msgCount));
    return;
  }
  byte rcvCmd = LoRa.read();    // replied command
  if (rcvCmd != cmdExpected) {
    DEBUGln("Invalid Reply: " + String(rcvCmd) + ", Expected: " + String(cmdExpected));
    return;
  }
  rcvData = LoRa.read();

  lastRSSI =  LoRa.packetRssi();
  lastSNR = LoRa.packetSnr();
  lastTurnaround = millis() - lastSendTime;
  lastFrequencyError = LoRa.packetFrequencyError();
  DEBUGln("Frequency Error: " + String(lastFrequencyError));
  DEBUGln("Working Frequency before update: " + String(workFrequency));
  workFrequency = workFrequency + lastFrequencyError;
  DEBUGln("Working Frequency after update: " + String(workFrequency));
  //  LoRa.setFrequency(workFrequency);
  //  delay(20);
  wasReceived = true;

  //Here we are if the received message is CORRECT
  DEBUGln("Reply number: " + String(rcvCount));
  DEBUGln("Reply command: " + String(rcvCmd));
  DEBUGln("Reply Data: " + String(rcvData));

  DEBUGln("RSSI: " + String(lastRSSI));
  DEBUGln("Snr: " + String(lastSNR));
  DEBUGln("Turnaround: " + String(lastTurnaround));
  DEBUGln("Frequency Error: " + String(lastFrequencyError));
  DEBUGln("Working Frequency: " + String(workFrequency));
  DEBUGln("Receive Message Done!");
}//void onReceive(int packetSize)

bool commSession( byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes ) {
  DEBUGln("commSession()");
  wasReceived = false;
  do {
    EVERY_MS(waitMilliseconds) {
      cmdExpected = expectedReply;
      sendMessage(msgCmd, sndData);
      doTimes--;
      DEBUGln("Comm tries left: " + String(doTimes));
    }
  } while ((doTimes > 0) && (!wasReceived));
  pingTimer = millis(); //refresh the ping timer after every communication
  return wasReceived;

}//commSession(...)

bool pairing() {
  DEBUGln("pairing()");
  workFrequency = CALL_FQ;
  LoRa.setFrequency(workFrequency);
  expectedTimeout = DEFAULT_TIMEOUT;
  //BROADCAST PARING, receive Pairing OK and address

  DEBUGln(" == CMD_PAIRING == ");
  flashLedParing();
  if (!commSession(CMD_PAIRING, CMD_PAIRING, CMD_PAIRING_OK, PAIRING_TIMEOUT, PAIRING_START_ATTEMPTS)) return false;
  //in fact, Pairing occures again after  failure...

  //Send received ADDR as data and expect msg with Addr_OK
  workAddress = rcvData; //In prev. connection, RX replied with her address
  DEBUGln(" == CMD_ADDR == ");
  flashLedParing();
  expectedTimeout = 3 * lastTurnaround;
  if (!commSession(CMD_ADDR, workAddress, CMD_ADDR_OK, expectedTimeout, PAIRING_COMM_ATTEMPTS)) return false;

  //Send Channel and expect Channel OK
  expectedTimeout = 3 * lastTurnaround;
  workChannel = workingChannel();
  DEBUGln(" == CMD_CHANNEL == ");
  flashLedParing();
  if (!commSession(CMD_CHANNEL, workChannel, CMD_CHANNEL_OK, expectedTimeout, PAIRING_COMM_ATTEMPTS)) return false;
  if (rcvData != workChannel) return false;

  //Send Start msg and expect Start OK
  workFrequency = frequencyByChannel(workChannel);
  //  LoRa.setFrequency(workFrequency);
  DEBUGln("Frequency: " + String(workFrequency));
  expectedTimeout = 3 * lastTurnaround;
  DEBUGln(" == CMD_START == ");
  flashLedParing();
  if (!commSession(CMD_START, CMD_START, CMD_START_OK, expectedTimeout, PAIRING_COMM_ATTEMPTS)) return false;
  if (rcvData != CMD_START_OK) return false; //дополнительная проверка

  //Pairing is done OK at this point
  DEBUGln(" == PARING OK != = ");
  flashLedParingOK();
  return true;
}//void pairing()
