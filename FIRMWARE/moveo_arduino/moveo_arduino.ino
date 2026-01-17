/**

  Moveo

  - Copyright (C) 2024 - PRESENT  rookidroid.com
  - E-mail: info@rookidroid.com
  - Website: https://rookidroid.com/

                        **
                       ****
                        **
                        **
                        **
                        **

        **********************************
      **************************************
     ****************************************
     ********      ************      ********
     *******        **********        *******
     *******        **********        *******
     ********      ************      ********
     ****************************************
     ****************************************
     ****************************************
     ****************************************


            **************************

                ******************

*/

/** Stepper Motor */
#include "FastAccelStepper.h"

/** Servo */
#include <ESP32Servo.h>

/** WiFi */
#include <AsyncUDP.h>
#include <WiFi.h>

/** OTA */
#include <ArduinoOTA.h>

/** Joint Stepper Configuration */
#define J1_DIR 13
#define J1_STEP 12

#define J2_DIR 14
#define J2_STEP 27

#define J3_DIR 26
#define J3_STEP 25

#define J4_DIR 33
#define J4_STEP 32

#define J5_DIR 15
#define J5_STEP 2

FastAccelStepperEngine engine = FastAccelStepperEngine();

FastAccelStepper *j1_stepper = NULL;
FastAccelStepper *j2_stepper = NULL;
FastAccelStepper *j3_stepper = NULL;
FastAccelStepper *j4_stepper = NULL;
FastAccelStepper *j5_stepper = NULL;

/** Hand Servo Configuration */
#define servoPin 4
const int MID = 1500;
const int MIN = 700;
const int MAX = 2300;

Servo hand_servo;

/** WiFi Configurations */
#ifndef APSSID
#define APSSID "moveo"
#define APPSK "moveo_1234"
#endif

const char *ssid = APSSID;
const char *password = APPSK;
AsyncUDP udp_socket;

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
      // using SPIFFS.end()
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
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

  ArduinoOTA.begin();

  pinMode(J1_DIR, OUTPUT);
  pinMode(J1_STEP, OUTPUT);
  pinMode(J2_DIR, OUTPUT);
  pinMode(J2_STEP, OUTPUT);
  pinMode(J3_DIR, OUTPUT);
  pinMode(J3_STEP, OUTPUT);
  pinMode(J4_DIR, OUTPUT);
  pinMode(J4_STEP, OUTPUT);
  pinMode(J5_DIR, OUTPUT);
  pinMode(J5_STEP, OUTPUT);

  hand_servo.attach(servoPin, MIN, MAX);
  hand_servo.writeMicroseconds(MID);

  engine.init();
  j1_stepper = engine.stepperConnectToPin(J1_STEP);
  if (j1_stepper) {
    j1_stepper->setDirectionPin(J1_DIR);

    j1_stepper->setSpeedInHz(3000);    // 3000 steps/s
    j1_stepper->setAcceleration(800);  // 800 steps/s²
  }
  //  j1_stepper->moveTo(1000, true);
  //  j1_stepper->moveTo(-1000, true);
  //  j1_stepper->moveTo(0, true);

  j2_stepper = engine.stepperConnectToPin(J2_STEP);
  if (j2_stepper) {
    j2_stepper->setDirectionPin(J2_DIR);

    j2_stepper->setSpeedInHz(3000);    // 3000 steps/s
    j2_stepper->setAcceleration(800);  // 800 steps/s²
  }
  //  j2_stepper->moveTo(2000, true);
  //  j2_stepper->moveTo(-2000, true);
  //  j2_stepper->moveTo(0, true);

  j3_stepper = engine.stepperConnectToPin(J3_STEP);
  if (j3_stepper) {
    j3_stepper->setDirectionPin(J3_DIR);

    j3_stepper->setSpeedInHz(3000);    // 3000 steps/s
    j3_stepper->setAcceleration(800);  // 800 steps/s²
  }
  //  j3_stepper->moveTo(2000, true);
  //  j3_stepper->moveTo(-2000, true);
  //  j3_stepper->moveTo(0, true);

  j4_stepper = engine.stepperConnectToPin(J4_STEP);
  if (j4_stepper) {
    j4_stepper->setDirectionPin(J4_DIR);

    j4_stepper->setSpeedInHz(3000);    // 3000 steps/s
    j4_stepper->setAcceleration(800);  // 800 steps/s²
  }
  //  j4_stepper->moveTo(2000, true);
  //  j4_stepper->moveTo(-2000, true);
  //  j4_stepper->moveTo(0, true);

  j5_stepper = engine.stepperConnectToPin(J5_STEP);
  if (j5_stepper) {
    j5_stepper->setDirectionPin(J5_DIR);

    j5_stepper->setSpeedInHz(3000);    // 3000 steps/s
    j5_stepper->setAcceleration(800);  // 800 steps/s²
  }
  //  j5_stepper->moveTo(2000, true);
  //  j5_stepper->moveTo(-2000, true);
  //  j5_stepper->moveTo(0, true);
}

void loop() {
  ArduinoOTA.handle();
}
