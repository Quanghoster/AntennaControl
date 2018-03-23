// test changes


#include <Arduino.h>

#include <Wire.h>
#include <QMC5883L.h>
#include <Stepper.h>
#include "config.h"
#include "Command.h"

// OTA
#include <WiFi.h>
#include <WiFiMulti.h>

WiFiMulti wifiMulti;
#define MAX_SRV_CLIENTS 1

#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
// OTA

WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];



/* Assign a unique ID to this sensor at the same time */
QMC5883L compass;


// for your motor
const int EL_StepsPerRevolution = EL_SPR;  // change this to fit the number of steps per revolution
const int AZ_StepsPerRevolution = AZ_SPR;  // change this to fit the number of steps per revolution

// initialize the stepper library on pins 8 through 11:
Stepper EL_Stepper(EL_StepsPerRevolution, EL_A, EL_B, EL_C, EL_D);
Stepper AZ_Stepper(AZ_StepsPerRevolution, AZ_A, AZ_B, AZ_C, AZ_D);

const byte EL_SensePin1 = EL_SENSE_1;
const byte EL_SensePin2 = EL_SENSE_2;

const byte AZ_SensePin1 = AZ_SENSE_1;
const byte AZ_SensePin2 = AZ_SENSE_2;

volatile int EL_Count1 = 0;
volatile int EL_Count2 = 0;
volatile int AZ_Count1 = 0;
volatile int AZ_Count2 = 0;

volatile int interruptCounter = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


int totalInterruptCounter;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/*
Interrupt Service Routines for AZ and EL sensors

*/

void IRAM_ATTR EL_SensePin1_ISR() {
  portENTER_CRITICAL_ISR(&mux);
  EL_Count1++;
  portEXIT_CRITICAL_ISR(&mux);
}
void IRAM_ATTR EL_SensePin2_ISR() {
  portENTER_CRITICAL_ISR(&mux);
  EL_Count2++;
  portEXIT_CRITICAL_ISR(&mux);
}
void IRAM_ATTR AZ_SensePin1_ISR() {
  portENTER_CRITICAL_ISR(&mux);
  AZ_Count1++;
  portEXIT_CRITICAL_ISR(&mux);
}
void IRAM_ATTR AZ_SensePin2_ISR() {
  portENTER_CRITICAL_ISR(&mux);
  AZ_Count2++;
  portEXIT_CRITICAL_ISR(&mux);
}

// process the current command instruction



volatile float azimuth = 0.0;

void compassPoll( void * parameter )
{
  for (;;) {
    delay(1000);
    // update compass heading

  float a;
  a = compass.readHeading();
  azimuth = a;
  
  }
}
TaskHandle_t Task1, Task2;
SemaphoreHandle_t baton;
void setup()
{
  Serial.begin(115200);
  
    Serial.println("Booting");


 /* initialise telnet server */
  wifiMulti.addAP(ssid, password);
  wifiMulti.addAP("ssid_from_AP_2", "your_password_for_AP_2");
  
 Serial.println("Connecting Wifi ");
  for (int loops = 10; loops > 0; loops--) {
    if (wifiMulti.run() == WL_CONNECTED) {
      Serial.println("");
      Serial.print("WiFi connected ");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      break;
    }
    else {
      Serial.println(loops);
      delay(1000);
    }
  }
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("WiFi connect failed");
    delay(1000);
    ESP.restart();
  }

  server.begin();
  server.setNoDelay(true);

ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

    
    /* Initialise the sensor */
  pinMode(19,INPUT_PULLUP);
  pinMode(21,INPUT_PULLUP);
  pinMode(22,INPUT_PULLUP);
  Wire.begin();
  compass.init();
  compass.setMode(Mode_Continuous,ODR_200Hz,RNG_8G,OSR_512);

  pinMode(EL_SensePin1, INPUT_PULLUP);
  pinMode(EL_SensePin2, INPUT_PULLUP);
  pinMode(AZ_SensePin1, INPUT_PULLUP);
  pinMode(AZ_SensePin2, INPUT_PULLUP);

  EL_Stepper.setSpeed(30);
  AZ_Stepper.setSpeed(30);

  // attach interrupt_isrs to EL_SENSE pins
  attachInterrupt(digitalPinToInterrupt(EL_SensePin1), EL_SensePin1_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(EL_SensePin2), EL_SensePin2_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(AZ_SensePin1), AZ_SensePin1_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(AZ_SensePin2), AZ_SensePin2_ISR, FALLING);
  
  xTaskCreatePinnedToCore(compassPoll,"compassTask", 1000, NULL,1, &Task1,1);

 baton = xSemaphoreCreateMutex();
  
/*

  
SemaphoreHandle_t baton;
 baton = xSemaphoreCreateMutex();
  
    */

  Serial.println("Ready");
  Serial.print(">");
  Serial.println("Starting OTA");
  ArduinoOTA.setHostname("AntennaController");
  ArduinoOTA.begin();

}

int stepCount = 0;
bool enableSlowStep = false;


void Zenith()
{
  bool HALT = false;
  // step one step:
  while (!HALT)
  {
    EL_Stepper.step(5);
    Serial.print("steps:");
    Serial.println(stepCount);
    stepCount += 5;
    delay(500);
    if (EL_Count1 > 0 || EL_Count2 > 0)
    {
      portENTER_CRITICAL(&mux);
      EL_Count1 = 0;
      EL_Count2 = 0;
      portEXIT_CRITICAL(&mux);
    }
    else
    {
      Serial.println("Limit reached");
      HALT = true;
    }
  }

}

int readline(int readch, char *buffer, int len)
{
  static int pos = 0;
  int rpos;

  if (readch > 0) {
    switch (readch) {
      case '\n': // Ignore new-lines
        break;
      case '\r': // Return on CR
        rpos = pos;
        pos = 0;  // Reset position index ready for next time
        return rpos;
      default:
        if (pos < len - 1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  // No end of line has been found, so return -1.
  return -1;
}

int currentTime = 0;

void processCommand(Stream *stream);
void printReport(Stream *stream);



// need to figure out what's happening with the Elevation steppermotor
void loop()
{
  EL_Stepper.step(10);
  delay(100);
  ArduinoOTA.handle();
}
void loop2()
{
  //Serial.println("Checking OTA");
  ArduinoOTA.handle();
  uint8_t i;
  if (wifiMulti.run() == WL_CONNECTED) {
    //check if there are any new clients
    if (server.hasClient()){
      for(i = 0; i < MAX_SRV_CLIENTS; i++){
        //find free/disconnected spot
        if (!serverClients[i] || !serverClients[i].connected()){
          if(serverClients[i]) serverClients[i].stop();
          serverClients[i] = server.available();
          if (!serverClients[i]) Serial.println("available broken");
          Serial.print("New client: ");
          Serial.print(i); Serial.print(' ');
          Serial.println(serverClients[i].remoteIP());
          break;
        }
      }
      if (i >= MAX_SRV_CLIENTS) {
        //no free/disconnected spot so reject
        server.available().stop();
      }
    }
    //check clients for data
    processCommand(&Serial);
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (serverClients[i] && serverClients[i].connected()){
        if(serverClients[i].available()){
          //get data from the telnet client and push it to the UART
          while(serverClients[i].available()) 
          {
            // process the command
             processCommand(&serverClients[i]);
          }
        }
      }
      else {
        if (serverClients[i]) {
          serverClients[i].stop();
        }
      }
    }
    if(millis() > currentTime + 1000)
    {
      printReport(&Serial);
      for(i = 0; i < MAX_SRV_CLIENTS; i++){
        printReport(&serverClients[i]);
      }
      currentTime = millis();
      if(enableSlowStep)
      {
        EL_Stepper.step(1);  
      }
    }
  

  }
  else {
    Serial.println("WiFi not connected!");
    for(i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (serverClients[i]) serverClients[i].stop();
    }
    delay(1000);
  }
} 


void printReport(Stream *stream)
{
  stream->print("AZ:");stream->print(azimuth);
  int val;
 
  portENTER_CRITICAL(&mux);
  val = EL_Count1;
  portEXIT_CRITICAL(&mux);
  stream->print(" E1:");stream->print(val);
  
  portENTER_CRITICAL(&mux);
  val = EL_Count2;
  portEXIT_CRITICAL(&mux);
  
  stream->print(" E2:");stream->print(val);
  
  portENTER_CRITICAL(&mux);
  val = AZ_Count1;
  portEXIT_CRITICAL(&mux);
  
  stream->print(" A1:");stream->print(val);
  
  portENTER_CRITICAL(&mux);
  val = AZ_Count2;
  portEXIT_CRITICAL(&mux);
  
  stream->print(" A2:");stream->print(val);
  
  stream->println("");
  stream->flush();
}

void processCommand(Stream *stream)
{
  static char buffer[80];
  if (readline(stream->read(), buffer, 80) > 0) {
    stream->println(buffer);
    if (strcmp(buffer, "Z") == 0)
    {
      stream->println("OK");
      Zenith();
    }
    else if (strcmp(buffer, "S") == 0)
    {
      enableSlowStep = !enableSlowStep;
    }
    else if (strcmp(buffer, "8") == 0)
    {
      EL_Stepper.step(EL_SPR);
    }
    else if (strcmp(buffer, "2") == 0)
    {
      EL_Stepper.step(-EL_SPR);
    }
    else if (strcmp(buffer, "4") == 0)
    {
      AZ_Stepper.step(AZ_SPR);
    }
    else if (strcmp(buffer, "6") == 0)
    {
      AZ_Stepper.step(-AZ_SPR);
    }
    else if (strcmp(buffer, "R") == 0)
    {
      ESP.restart();
    }  
    }
}


