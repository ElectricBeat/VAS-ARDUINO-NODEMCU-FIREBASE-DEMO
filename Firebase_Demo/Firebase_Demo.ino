/*
  Rui Santos
  Complete project details at our blog.
    - ESP32: https://RandomNerdTutorials.com/esp32-firebase-realtime-database/
    - ESP8266: https://RandomNerdTutorials.com/esp8266-nodemcu-firebase-realtime-database/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  Based in the RTDB Basic Example by Firebase-ESP-Client library by mobizt
  https://github.com/mobizt/Firebase-ESP-Client/blob/main/examples/RTDB/Basic/Basic.ino
*/

#include <Time.h>

#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

#include "addons/TokenHelper.h"                                                                 // Provide the token generation process info.
#include "addons/RTDBHelper.h"                                                                  // Provide the RTDB payload printing info and other helper functions.

#define WIFI_SSID "Vasantha-wifi"                                                               // Insert your network credentials
#define WIFI_PASSWORD "Vasantha@123"

#define API_KEY "AIzaSyDM0wJTWA-gsKt-hdw0FcOYYAyzVsgX_Jg"                                       // Insert Firebase project API Key

#define DATABASE_URL "https://test1-3bf8c-default-rtdb.asia-southeast1.firebasedatabase.app"    // Insert RTDB URLefine the RTDB URL

FirebaseData fbdo;                                                                              // Define Firebase Data object

FirebaseAuth auth;
FirebaseConfig config;

volatile unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

#include <OneWire.h>
#include <DallasTemperature.h>
const int oneWireBus = 4;                                                                       // GPIO where the DS18B20 is connected to
OneWire oneWire(oneWireBus);                                                                    // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);                                                            // Pass our oneWire reference to Dallas Temperature sensor 

const int updateInterval = 60000;                                                               // Update interval in milliseconds (1 minute)
unsigned long  Interval;
unsigned long lastUpdateTime = 0;                                                               // Stores the last update time
const int sensorPin = 14;
volatile int pulseCount = 0;
volatile unsigned long previousTime = 0;
volatile unsigned long timeDifference = 1;
volatile float Ans;
int RPM,SPM;
unsigned long currentTime;

void setup()
{
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  config.api_key = API_KEY;                                                                     // Assign the api key (required)

  config.database_url = DATABASE_URL;                                                           // Assign the RTDB URL (required)

  if (Firebase.signUp(&config, &auth, "", ""))                                                  // Sign up
  {
    Serial.println("ok");
    signupOK = true;
  }
  else
  {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback;                                           // Assign the callback function for the long running token generation task
                                                                                                //see addons/TokenHelper.h
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  sensors.begin();                                                                              // Start the DS18B20 sensor
  pinMode(sensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(sensorPin), countPulse, RISING);
}

void loop()
{
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);                                              // Field 1: Temperature
  Serial.print(temperatureC);
  Serial.println("ºC");

  Serial.print("timeDifference: ");
  Serial.print(timeDifference);
  Serial.print("\t");
  RPM = (((60*1000)/timeDifference));                                                           // Field 2: RPM
  if(RPM <= 300)
  {
    SPM = RPM;
  }
  Serial.print("RPM: ");
  Serial.print(SPM);
  Serial.print("\t");

  if(RPM >= 500)
  {
    Serial.print("RPM: ");
    Serial.print(RPM);                                                                          // Field 2: RPM
    Serial.print("\t");
    RPM = 0;
  }

  if (Firebase.ready() && signupOK && (micros() - sendDataPrevMillis > 3000000 || sendDataPrevMillis == 0))
  {
    sendDataPrevMillis = micros();

    if (Firebase.RTDB.setInt(&fbdo, "test/temperature C", temperatureC))                                  // Write an Int number on the database path test/int
    {
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    }
    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "test/SPM", SPM))                                       // Write an Float number on the database path test/float
    {
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    }
    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
  }

  unsigned long NowcurrentTime = millis();
  Interval = NowcurrentTime - previousTime;
  if(Interval >= updateInterval)
  {
    Interval = 0;
    previousTime = NowcurrentTime;

    Serial.print("Time out: RPM is restart : 000");
    timeDifference = 1;
    Ans = 0;
    RPM = 0;
  }

  Serial.println(" ");
}

ICACHE_RAM_ATTR void countPulse() 
{
  currentTime = millis();
  if (currentTime > previousTime) 
  {
    timeDifference = currentTime - previousTime;
  }
  else 
  {
    timeDifference = (unsigned long)(millis() + (65535 - previousTime));                        // Account for timer overflow
  }
  previousTime = currentTime;                                                                   // Update previousTime for the next pulse
}
