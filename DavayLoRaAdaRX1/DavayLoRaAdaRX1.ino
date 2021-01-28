/*
  LoRa Adafruit Feather32u4 433MHz module
  Davay RX, based on Duplex communication wth callback example
  Listens for a message from TX, about the button pressed/released.
  Turnes LED|Buzzer|Vibro  ON/OFF and replies to TX when receives.
  Для каждой пары TX-RX надо поменять в коде частоту, делитель батарейки и выбрать MY_ADDRESS
  Для изменений искать (Ctrl-F), пометив слово: МЕНЯТЬ 
*/

#include <SPI.h>              // include libraries
#include <LoRa.h>
//#include <EEPROM.h>

#ifdef ARDUINO_SAMD_MKRWAN1300  //not my code :)
#error "This code is not compatible with the Arduino MKR WAN 1300 board!"
#endif

// Дебагирование с компьютером : раскомментить для использования 1 строчку:
//#define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.print(String(millis())+" "+x)
#define DEBUGln(x) Serial.println(String(millis())+" "+x)
#else
#define DEBUG(x)
#define DEBUGln(x)
#endif

//==== MILLIS TIMER MACRO ====
// воспомогательное макро
// performs the {subsequent} code once and then, if needed, again after each x ms
#define EVERY_MS(x) \
  static uint32_t tmr;\
  bool flg = millis() - tmr >= (x);\
  if (flg) tmr = millis();\
  if (flg)
//===========================

#define CALL_FQ 434E6                       //рабочая частота.
//Это остатки от программирования «паринга», тут остались просто на всякий случай
#define MIN_FQ 433.05E6                     //минимальная частота диапазона.
#define MAX_FQ 434.79E6                     //максимальная рабочая частота.
#define CHANNEL_WIDTH 1E5                 // 100 или 200 KHz
#define NUM_LORA_CHANNELS (MAX_FQ - MIN_FQ)/CHANNEL_WIDTH   //количество каналов столько кГц
long minFQ = round(MIN_FQ / 1.0E5) * 1.0E5;
//TODO: сделать выбор рабочего канала, вместо частоты:
#define WORKING_CHANNEL 5
#define BROADCAST_ADDRESS 0xFF


#define PING_TIMEOUT 5000  //ms
#define PING_FLASH_ACTIVE 200  //ms
#define PING_FLASH_PAUSE 400  //ms
#define BATTERY_MIN 3.3   //Volt min. - ниже этого программа пытается не работать, хотя используемые батарейки самоотключаются при 2.7В

#define PIN_SIGNAL_LED  6  // Номер пина для вывода сигнала для ЛЕДа
#define PIN_SIGNAL_BUZZERS  5  // Номер пина для вывода сигнала для Баззера и Вибро
#define PIN_LED  LED_BUILTIN  // Номер пина, к которому подключен вывод статусного ЛЕД-а (LED_BUILTIN=13) 
#define PIN_BATTERY 9  // Номер пина Адафрута для измерения батарейки
#define PIN_BATTERY_LED LED_BUILTIN  // Номер пина для показа напряжения батарейки

//МЕНЯТЬ Коэффициент делителя для измерения батарейки. 
//Для Adafruit поставить 2, для BSFrance поставить 1.27
float batteryVoltageMultiplier = 2;

//Дефолтовые значения для SPI библиотеки: 10, 9, 2. Для наших плат действуют такие: 
const int csPin = 8;          // LoRa radio chip select
const int resetPin = 4;       // LoRa radio reset
const int irqPin = 7;         // change for your board; must be a hardware interrupt pin

//Практически, описание протокола:
#define CMD_PAIRING     200 //TX передаёт команду/команду с бродкастным адресом
#define CMD_PAIRING_OK  201 //RX отвечает /свой зашитый адрес с бродкастным адресом
#define CMD_ADDR        202 //TX передаёт по адресу полученный адрес/адрес
#define CMD_ADDR_OK     203 //RX отвечает /адрес с адресом
#define CMD_CHANNEL     204 //TX передаёт свой будущий /канал
#define CMD_CHANNEL_OK  205 //RX отвечает, что переключается/канал
#define CMD_START       206 //TX передаёт на канале старт/старт
#define CMD_START_OK    207 //RX отвечает на канале ok/ok - паринг закончился
#define CMD_SIGNAL         208 //TX передаёт сигнал на изменение состояния лед-а
#define CMD_SIGNAL_OK      209 //RX подтверждает
#define CMD_PING        212 //TX пингует периодически с /состоянием леда
#define CMD_PONG        213 //RX отвечает с состоянием леда
#define CMD_PING_OK     213 //то же что предыдущее

//МЕНЯТЬ синхронно для TX и RX в диапазоне 0-254 
#define MY_ADDRESS      78

byte workAddress = MY_ADDRESS;  // address of connection
byte msgNumber = 0;                    // = number of the received message: reply always this number
byte rcvAddress = 0;          // received address
byte rcvCount = 0;            // received Number
byte rcvCmd = 0;              // received command
byte rcvData = 0;                  // received data
byte sndCmd = CMD_PONG;              // outgoing command Default = PING
byte sndData;                         // additional data byte sent
bool signalStatus;                     //last received TX Button status
byte workChannel;                      //channel to send to RX while paring and work on it
unsigned long workFrequency; //working Frequency

unsigned long lastSendTime = 0;                // last send time
int lastRSSI;
float lastSNR;
unsigned long lastTurnaround;         // round-trip time between tx and rx
long lastFrequencyError;

unsigned long timeOutFlashPhase;
unsigned long timeOutFlashTime;
bool timeOutFlash;

int pwmledBrightness = 20;           // 0 - 255 - Яркость большого леда

//МЕНЯТЬ ? Cutoff settings:
unsigned long cutoffTimer = 0;
#define CUTOFF_TIME 2000  //ms - максимальное время работы сигнала

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
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  //INIT PINS
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SIGNAL_BUZZERS, OUTPUT);
  pinMode(PIN_SIGNAL_LED, OUTPUT);
  analogWrite(PIN_SIGNAL_LED, 0); //just in case - switch off FB on big led
  digitalWrite(PIN_BATTERY_LED, 0);

  // два раза показываем заряд батарейки:
  showBatteryVoltage();
  showBatteryVoltage();

//МЕНЯТЬ рабочую частоту (синхронно на TX и RX!)
//Желательно сильно не уходить от значения 434E6 ()
//просто добавлять-убавлять десятые, например: 433.9E6, 433.8E6, или 434.1E6, 434.2E6  
 workFrequency = 434E6;

  if (!LoRa.begin(workFrequency)) {             // initialize radio at CALL Frequency
    DEBUGln("LoRa init failed. Check your connections.");
    while (true) {
      flashlLedError();    // if failed, do nothing
    }
  }

  setLoRaParams();

  LoRa.onReceive(onReceive);
  //  LoRa.onTxDone(onTxDone);
  LoRa.receive(); //Always listen by default

  DEBUGln("Setup complete");
}//setup      //===================END SETUP===============================

void loop() { //  ===!!!===!!!===!!!===LOOP===!!!===!!!===!!!===!!!===!!!===

  processTimeOut();  //see if we haven't lost connection to TX
  processCommand();
  processCutoff();
  EVERY_MS(100000) {
    processBattery();
  }

}//loop()         ===!!!===!!!===!!!===END LOOP===!!!===!!!===!!!===!!!===!!!===

void   processTimeOut() {
  if ((millis() - lastSendTime) > PING_TIMEOUT) { // if long time no signal from TX
    signalStatus = false;
    if ((millis() - timeOutFlashTime) > timeOutFlashPhase) {
      timeOutFlash = !timeOutFlash;
      if (timeOutFlash) {
        timeOutFlashPhase = PING_FLASH_ACTIVE;
      } else {
        timeOutFlashPhase = PING_FLASH_PAUSE;
      }
      timeOutFlashTime = millis();
      updateLed(timeOutFlash);
    }
  }
}// processTimeOut()

void processCommand() {

  switch (rcvCmd) {
    case CMD_SIGNAL:
      DEBUGln("===CMD_SIGNAL===");
      signalStatus = rcvData;
      processSignal();
      sendMessage(rcvAddress, msgNumber, CMD_SIGNAL_OK, signalStatus);
      break;
    case CMD_PING:
      DEBUGln("===CMD_PING===");
      signalStatus = rcvData;
      processSignal();
      updateLed(true);
      unsigned long flashStatus = millis();
      sendMessage(rcvAddress, msgNumber, CMD_PONG, signalStatus);
      unsigned long restDelay = 100 - (millis() - flashStatus);
      if (restDelay > 0) {
        delay(restDelay);
      }
      updateLed(false);
  }
  rcvCmd = 0;
} //void processCommand()


void  processSignal() {
  cutoffTimer = millis();
  analogWrite(PIN_SIGNAL_LED, signalStatus * pwmledBrightness);
  digitalWrite(PIN_SIGNAL_BUZZERS, signalStatus);
}

void processCutoff() {
  if (millis()-cutoffTimer>CUTOFF_TIME) {
    signalStatus = 0;
    analogWrite(PIN_SIGNAL_LED, 0);
    digitalWrite(PIN_SIGNAL_BUZZERS, 0);
  }
}

void updateLed(bool ledStatus) { // turn ON or OFF the status LED
  digitalWrite(PIN_LED, ledStatus);
  DEBUGln("updateLed: " + String(ledStatus));
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

long frequencyByChannel(byte numChannel) {
  DEBUGln("frequencyByChannel(byte numChannel)");
  if (numChannel > NUM_LORA_CHANNELS) {
    DEBUGln("Invalid Channel: " + String(numChannel));
    return CALL_FQ;
  }
  return (minFQ + numChannel * CHANNEL_WIDTH);
}
// done frequencyByChannel(byte numChannel)

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
  DEBUGln(("sendMessage done: ") + String(msgAddr)\
          + " " + String(msgNumber) + " " + String(msgCmd) + " " + String(msgData));
}// void sendMessage(byte messageByte)

//void onTxDone() {
//  DEBUGln("onTxDone()");
//  lastSendTime = millis();            // timestamp the message
//  LoRa.receive();                     // go back into receive mode
//}

void onReceive(int packetSize) {
  DEBUGln("Package Received!");
  if (packetSize != 4) { //нас интересуют только пакеты в 4 байта
    DEBUGln("Wrong Packet Size: " + String(packetSize));
    return;          // not our packet, return
  }

  // read packet header bytes:
  rcvAddress = LoRa.read();          // received address
  rcvCount = LoRa.read();            // received Number
  rcvCmd = LoRa.read();              // received command
  rcvData = LoRa.read();                  // received data

#ifdef DEBUG_ENABLE  //только имеет смысл для дебагинга
  lastRSSI =  LoRa.packetRssi();
  lastSNR = LoRa.packetSnr();
  lastTurnaround = millis() - lastSendTime;
  lastFrequencyError = LoRa.packetFrequencyError();
#endif

  if (rcvAddress != workAddress) {
    DEBUGln("Wrong Address: " + String(rcvAddress) + " - " +  String(workAddress));
    return;
  }

  msgNumber = rcvCount;

  DEBUGln("RSSI: " + String(lastRSSI));
  DEBUGln("Snr: " + String(lastSNR));
  DEBUGln("Turnaround: " + String(lastTurnaround));
  DEBUGln("Frequency Error: " + String(lastFrequencyError));
  DEBUGln("Received Message: "  + String(rcvAddress) + " " + String(rcvCount)\
          +" " + String( rcvCmd) + " " + String( rcvData));

}//void onReceive(int packetSize)

void processBattery() {

  if (batteryVoltage() < BATTERY_MIN) {
    stopWorking();
  }

}

void showBatteryVoltage() {
  float voltage = batteryVoltage();
  delay(1000);
  if (voltage > 3.5)   flashBatteryOnce(); //1 раз
  if (voltage > 3.6)   flashBatteryOnce(); //2 раз
  if (voltage > 3.7)   flashBatteryOnce(); //3 раз
  if (voltage > 3.8)   flashBatteryOnce(); //4 раз
  if (voltage > 4.0)   flashBatteryOnce(); //5 раз
  delay(1000);
}

void flashBatteryOnce() {
  digitalWrite(PIN_BATTERY_LED, 1);
  delay(250);
  digitalWrite(PIN_BATTERY_LED, 0);
  delay(250);
}

float batteryVoltage() {
  float measuredvbat = analogRead(PIN_BATTERY);
  measuredvbat *= batteryVoltageMultiplier;    // multiply according to the used board divider
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  DEBUGln("Battery Voltage: " + measuredvbat);
  return measuredvbat;
}

void stopWorking() {
  flashlLedBattery();
  while (1) {
  }
}

void flashlLedBattery() { //flash 3 times total 1.5 sec
  DEBUGln("flashlLedError()");
  bool flash = false;
  for (int i = 0; i < 8; i++) {
    flash = !flash;
    digitalWrite(PIN_BATTERY_LED, flash);
    delay(200);
  }
  delay(1000);
}
