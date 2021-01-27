/*
  LoRa Adafruit Feather32u4 433MHz module
  Davay TX, based on Duplex communication with callback example
  Sends a message when button is pressed/released or pings on timer
  Waits for reply on callback
  Для другого диапазона надо пересмотреть некоторые дефайны

*/
#include <SPI.h>              // include libraries
#include <LoRa.h>

#ifdef ARDUINO_SAMD_MKRWAN1300  //not my code - this is LoRa.h example suggested code :)
#error "This code is not compatible with the Arduino MKR WAN 1300 board!"
#endif

// Дебагирование: раскомментить для использования 1 строчку:
//#define DEBUG_ENABLE
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

//#define MIN_FQ 433.05E6                     //минимальная частота диапазона.
//#define MAX_FQ 434.79E6                     //максимальная рабочая частота.
//#define CHANNEL_WIDTH 1E5                 // поставить 100 или 200 KHz
//#define NUM_LORA_CHANNELS (MAX_FQ - MIN_FQ)/CHANNEL_WIDTH   //количество каналов столько кГц
//long minFQ = round(MIN_FQ / 1.0E5) * 1.0E5;
////TODO: сделать автоматический выбор рабочего канала:
//#define WORKING_CHANNEL 5                 //fixed. supposed controlled by jumpers or by scan
//#define BROADCAST_ADDRESS 0xFF

#define DEFAULT_TIMEOUT 300               //начальный таймаут для получения фидбяка (мс)
#define WORK_COMM_ATTEMPTS 3
#define PING_TIMEOUT 3000  //ms
#define PING_FLASH 100  //ms
#define PING_FLASH_PAUSE 400  //ms Не используется - тут для симметричности с RX
#define BATTERY_MIN 3.4   //Volt min.

#define PIN_BUTTON  6  // Номер пина Arduino, к которому подключен вывод кнопки (притянуто к 5в)
#define PIN_FB_LED  5  // Номер пина Arduino, к которому подключен вывод LED обратной связи
#define PIN_BATTERY 9  // Номер пина Адафрута для измерения батарейки
#define PIN_PWM_LED 11 // Номер пина для ШИМ большого ЛЕДа
#define PIN_BATTERY_LED LED_BUILTIN  // Номер Адафрута для измерения батарейки

//Коэффициент для батарейки. Для Adafruit поставить 2, для BSFrance поставить 1.27
float batteryVoltageMultiplier = 2;

//Дефолтовые: 10, 9, 2 для SPI библиотеки
const int csPin = 8;          // LoRa radio chip select
const int resetPin = 4;       // LoRa radio reset
const int irqPin = 7;         // change for your board; must be a hardware interrupt pin

//Практически, описание протокола (тут остался Паринг, но он не мешает):
#define CMD_PAIRING     200 //TX передаёт команду/команду с бродкастным адресом
#define CMD_PAIRING_OK  201 //RX отвечает /свой зашитый адрес с бродкастным адресом
#define CMD_ADDR        202 //TX передаёт полученный адрес/адрес с адресом :)
#define CMD_ADDR_OK     203 //RX отвечает /адрес с адресом
#define CMD_CHANNEL     204 //TX передаёт свой будущий /канал
#define CMD_CHANNEL_OK  205 //RX отвечает, что переключается/канал
#define CMD_START       206 //TX передаёт на канале старт/старт
#define CMD_START_OK    207 //RX отвечает на канале ok/ok - паринг закончился
#define CMD_SIGNAL         208 //TX передаёт сигнал на изменение состояния Сигнала
#define CMD_SIGNAL_OK      209 //RX подтверждает
#define CMD_PING        212 //TX пингует периодически с /состоянием леда
#define CMD_PONG        213 //RX отвечает с состоянием леда
#define CMD_PING_OK     213 //то же что предыдущее

//Specific for each RX:
#define MY_ADDRESS      78

byte workAddress = MY_ADDRESS;  // address of connection
byte msgCount = 0;                    // = number of outgoing message
byte sndCmd = CMD_PING;              // outgoing command Default = PING
byte sndData;                         // additional data byte sent
bool wasReceived = 0;                     //indication of reply message received
byte cmdExpected = CMD_PONG;          // expected command (Default = PONG)
byte rcvAddress = 0;          // received address
byte rcvCount = 0;            // received Number
byte rcvCmd = 0;                      // received command
byte rcvData;                         // additional data byte received
byte workChannel;                      //channel to send to RX while paring and work on it
unsigned long workFrequency = CALL_FQ; //working Frequency

long lastSendTime = 0;                // last send time
int lastRSSI;
float lastSNR;
unsigned long lastTurnaround = 100;         // round-trip time between tx and rx
long lastFrequencyError;
unsigned long expectedTimeout = DEFAULT_TIMEOUT;

bool currButtonState = 0;               //Current state of Button, initially OFF
bool prevButtonState = 0;               //Previous state of Button, initially OFF
bool firstPressButton = 0;              //Включается работа по первому нажатию

unsigned long pingTimer;
unsigned long pingFlashTimer;
bool pingFlash;

int fbledBrightness = 255;           // 0 - 255 - Яркость леда в кнопке
int pwmledBrightness = 15;           // 0 - 30 - Яркость большого леда (больше 30 - слишком ярко! и много потребляет )

void setup() {//=======================SETUP===============================

  // initialize serial
#ifdef DEBUG_ENABLE
  Serial.begin(9600);
  while (!Serial);
#endif

  DEBUGln("================================");
  DEBUGln("=========== START TX ===========");
  DEBUGln("Davay Ada-LoRa TX setup()");

  // override the library default CS, reset, and IRQ pins
  LoRa.setPins(csPin, resetPin, irqPin);  // set CS, reset, IRQ pin

  //INIT PINS for button and status LEDs:
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_FB_LED, OUTPUT);
  pinMode(PIN_PWM_LED, OUTPUT);
  analogWrite(PIN_FB_LED, 0); //just in case - switch off FB on big led
  analogWrite(PIN_PWM_LED, 0); //just in case - switch off PWM on big led
  digitalWrite(PIN_BATTERY_LED, 0);

  delay(1000);
  showBatteryVoltage();
  showBatteryVoltage();
  showBatteryVoltage();

  if (!LoRa.begin(CALL_FQ)) {             // initialize radio at CALL Frequency
    DEBUGln("LoRa init failed. Check your connections.");
    while (true) {
      flashlLedError();    // if failed, do nothing
    }
  }

  setLoRaParams();          //Tweak parameters for best communication

  LoRa.onReceive(onReceive);
  //  LoRa.onTxDone(onTxDone);
  LoRa.idle();              //Until we decide how to continue

  DEBUGln("LoRa TX init success.");

}//setup      //======================= SETUP ===============================

void loop() { //  ===!!!===!!!===!!!===!!!= LOOP =!!!===!!!===!!!===!!!===!!!===

  processButton();
  processPing();
  processCommand();
  EVERY_MS(100000) {
    processBattery();
  }

}//loop()         ===!!!===!!!===!!!===!!!= LOOP =!!!===!!!===!!!===!!!===!!!===

void   processButton() {
  currButtonState = !digitalRead(PIN_BUTTON); // Читаем состояние кнопки 1=нажата; 0=отпущена
  if (prevButtonState != currButtonState) {   //button pressed or released
    firstPressButton = true;
    DEBUGln("processButton(): " + String(currButtonState));
    prevButtonState = currButtonState;
    if (commSession( CMD_SIGNAL, currButtonState, CMD_SIGNAL_OK, 3 * lastTurnaround, WORK_COMM_ATTEMPTS )) {
      updateFBLed(currButtonState);
      updatePWMLed(currButtonState);
    }
    else {
      updatePWMLed(false);
      updateFBLed(false);
      flashlLedError();
    }
  }
}//void   processButton()

void processCommand() {
  if (wasReceived) {
    DEBUGln("======processCommand() - received");

    DEBUGln("Reply number: " + String(rcvCount));
    DEBUGln("Reply command: " + String(rcvCmd));
    DEBUGln("Reply Data: " + String(rcvData));

    DEBUGln("RSSI: " + String(lastRSSI));
    DEBUGln("Snr: " + String(lastSNR));
    DEBUGln("Turnaround: " + String(lastTurnaround));
    DEBUGln("Frequency Error: " + String(lastFrequencyError));
    DEBUGln("Working Frequency: " + String(workFrequency));
    DEBUGln("Receive Message Done!");
    workFrequency = workFrequency - lastFrequencyError / 2;
    DEBUGln("Working Frequency after update: " + String(workFrequency));
    LoRa.setFrequency(workFrequency);
    delay(30);
    wasReceived = false;
    DEBUGln("======processCommand() - done");
  }
}//void processCommand()


void   processPing() {
  if (!firstPressButton) return;
  if (pingFlash) {// ping reply was already received, FB led is on, processing FB led off only
    if ((millis() - pingFlashTimer) > PING_FLASH) { //end led flash
      DEBUGln("Ping LED OFF");
      pingFlash = false;
      updateFBLed(currButtonState);
    }
    return;  //??
  }
  if ((millis() - pingTimer) > PING_TIMEOUT) { // long time was no command - initiate ping
    DEBUGln("Start Ping");
    if (commSession( CMD_PING, currButtonState, CMD_PONG, 3 * lastTurnaround, WORK_COMM_ATTEMPTS )) {
      DEBUGln("Ping LED ON");
      updateFBLed(!currButtonState);
      pingFlash = true;
      pingFlashTimer = millis();
    }
    else {
      updateFBLed(false);
      flashlLedError();
    }
  }
}//void   processPing()

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

void updateFBLed(bool ledStatus) { // turn ON or OFF the Status LED
  //  digitalWrite(PIN_FB_LED, ledStatus);
  analogWrite(PIN_FB_LED, ledStatus * fbledBrightness);
  //  DEBUGln("updateFBLed(): " + String(ledStatus));
}

void updatePWMLed(bool ledStatus) { // turn ON or OFF the Status LED
  //  digitalWrite(PIN_FB_LED, ledStatus);
  analogWrite(PIN_PWM_LED, ledStatus * pwmledBrightness);
  //  DEBUGln("updatePWMLed(): " + String(ledStatus));
}

void flashlLedError() { //flash 3 times total 1.5 sec
  DEBUGln("flashlLedError()");
  bool flash = false;
  for (int i = 0; i < 8; i++) {
    flash = !flash;
    updateFBLed(flash);
    delay(100);
  }
  delay(200);
}

void setLoRaParams() {
  DEBUGln("setLoRaParams()");
  //Trying setting LoRa for Longest Range possible:
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
  LoRa.receive();                     // go back into receive mode
  lastSendTime = millis();            // timestamp the message
  DEBUGln("sendMessage done! " + String(workAddress)\
          + " " + String(msgCount) + " " + String(msgCmd) + " " + String(sndData));
}// void sendMessage(byte messageByte)

//void onTxDone() {
//  DEBUGln("onTxDone()");
//  lastSendTime = millis();            // timestamp the message
//  LoRa.receive();                     // go back into receive mode
//}

void onReceive(int packetSize) {
  DEBUGln("onReceive()");
  if (packetSize != 4) {
    DEBUGln("Invalid Packet Size: " + String(packetSize));
    return;          // not our packet, return
  }
  // read packet header bytes:
  rcvAddress = LoRa.read();          // replied address
  if (rcvAddress != workAddress) {
    DEBUGln("Invalid Address: " + String(rcvAddress) + ", Expected: " + String(workAddress));
    return;
  }
  rcvCount = LoRa.read();    // replied number of Message
  if (rcvCount != msgCount) {
    DEBUGln("Invalid Number: " + String(rcvCount) + ", Expected: " + String(msgCount));
    return;
  }
  rcvCmd = LoRa.read();    // replied command
  if (rcvCmd != cmdExpected) {
    DEBUGln("Invalid Reply: " + String(rcvCmd) + ", Expected: " + String(cmdExpected));
    return;
  }
  rcvData = LoRa.read();

  lastRSSI =  LoRa.packetRssi();
  lastSNR = LoRa.packetSnr();
  lastTurnaround = millis() - lastSendTime;
  lastFrequencyError = LoRa.packetFrequencyError();

  wasReceived = true;

}//void onReceive(int packetSize)

bool commSession( byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes ) {
  DEBUGln("commSession()");
  wasReceived = false;
  int totTimes = doTimes;
  do {
    EVERY_MS(waitMilliseconds) {
      cmdExpected = expectedReply;
      if (totTimes != doTimes) {
        DEBUGln("Comm tries left: " + String(doTimes));
      }
      sendMessage(msgCmd, sndData);
      doTimes--;
    }
  } while ((doTimes > 0) && (!wasReceived));
  pingTimer = millis(); //refresh the ping timer after every communication
  return wasReceived;

}//commSession(...)
