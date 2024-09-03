#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID "TMPLQ3apk_X1"
#define BLYNK_DEVICE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "DWezjfkzEM5F_7ZeFc-wYjf_Rb4BNrjd"
//------------------------------------------------------------------
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "TinyGPS++.h"
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

//------------------------------------------------------------------
char ssid[] = "Terhubung";
char pass[] = "Terhubung";

//const int RXPin = 4, TXPin = 5;
#define RXPin D2
#define TXPin D1
const uint32_t GPSBaud = 9600;
SoftwareSerial gps_module(TXPin, RXPin);

TinyGPSPlus gps; 

//Variable  to store the speed, no. of satellites, direction
float latitude = 0.00, lastlatitude = 0.00;
float longitude = 0.00, lastlongitude = 0.00;
float gps_speed;
int no_of_satellites;
String satellite_orientation;

char auth[] = BLYNK_AUTH_TOKEN;
// Your GPRS credentials. Leave empty, if missing user or pass
char apn[]  = "";
char user[] = "";
//char pass[] = "";

//------------------------------------------------------------------
BlynkTimer timer;

//------------------------------------------------------------------
#define RelayPin 12
#define BuzzerPin 13
#define SensorGetarPin 14
#define ButtonPin 15 // D8
//#define ButtonPin 9 // S2

#define EE_Size 5
#define RESET_EEPROM 1

#define EE_Status  3
#define EE_Deff  4

#define JarakMax 100.00 // 100 meter
#define GarisBujur 111320.00 //111.32 Km = 111320 meter
//------------------------------------------------------------------
uint8_t Status = 0;

//------------------------------------------------------------------
uint8_t GetarState = 0, BuzzState = 0;
uint8_t LastGetarState = 0;
unsigned long lastmillis = 0, lastmillisBuzz = 0;
boolean BuzzerON = false;
int buttonState = 0;
int lastButtonState = 0;

void setup()
{
  Serial.begin(9600);
  gps_module.begin(GPSBaud);
  EEPROM.begin(EE_Size);
  
  //--------------------------------------------------------------------
  pinMode(RelayPin, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);
  pinMode(SensorGetarPin, INPUT);
  pinMode(ButtonPin, INPUT);

  //--------------------------------------------------------------------
  //During Starting all Relays should TURN OFF
  digitalWrite(RelayPin, LOW);
  digitalWrite(BuzzerPin, LOW);
  
  //--------------------------------------------------------------------
  delay(1000);
  if(EEPROM.read(EE_Deff) != RESET_EEPROM)
  {
    EEPROM.write(EE_Deff, RESET_EEPROM); delay(5);
    EEPROM.write(EE_Status, 0); delay(5);
    EEPROM.commit();

    Serial.println(F("EE_Deff ")); Serial.println(EEPROM.read(EE_Deff));
  }
  
  Serial.print("Getar "); Serial.println(digitalRead(SensorGetarPin));
  Serial.print("EE_Deff "); Serial.println(EEPROM.read(EE_Deff));
  Serial.print("EE_Status "); Serial.println(EEPROM.read(EE_Status));

  delay(1000);
  Status = EEPROM.read(EE_Status);
  if(Status) { BuzzerON = true; digitalWrite(RelayPin, HIGH); }
    
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 8080);
  timer.setInterval(30000L, checkGPS);
}

void checkGPS()
{
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    Blynk.virtualWrite(V3, "GPS ERROR");
  }
}

void CekTombol()
{
  buttonState = digitalRead(ButtonPin);
  if (buttonState != lastButtonState) {
    if (buttonState == HIGH) {
        EEPROM.write(EE_Status, 0);
        EEPROM.commit();
        Serial.println(F("Status Reset"));
        Status = EEPROM.read(EE_Status);
        BuzzerON = 0;
        digitalWrite(RelayPin, LOW);
        digitalWrite(BuzzerPin, LOW);
    }
    delay(50);
  }
  lastButtonState = buttonState;
}

void loop()
{
  while (gps_module.available() > 0) 
  {
    if (gps.encode(gps_module.read()))
    displayInfo();
  }

  CekTombol();
  
  if(millis() - lastmillis >= 1000) {
    lastmillis = millis();
    GetarState = digitalRead(SensorGetarPin);
    Serial.print(F("Cek Getar ")); Serial.println(GetarState);
    if(GetarState != LastGetarState)
    {
      if(GetarState == HIGH)
      {
        BuzzerON = true;
        EEPROM.write(EE_Status, BuzzerON);
        EEPROM.commit();
        digitalWrite(RelayPin, HIGH);
        Status = EEPROM.read(EE_Status);
        Serial.println(F("Getar High "));
      }
      else
      {
        digitalWrite(BuzzerPin, LOW);
        digitalWrite(RelayPin, LOW);
        Serial.println(F("Getar Low "));
      }
    }
    LastGetarState = GetarState;
  }

  if(BuzzerON) {
    if(millis() -  lastmillisBuzz > 500) {
      lastmillisBuzz = millis();
      BuzzState = !BuzzState;
      digitalWrite(BuzzerPin, BuzzState);
      Serial.println(F("Buzzer ON"));
    }
  }
  
  Blynk.run();
  timer.run();
}

void displayInfo()
{
  if (gps.location.isValid()) 
  {
    //Storing the Latitude and Longitude
    latitude = (gps.location.lat());
    longitude = (gps.location.lng());  
    
    Blynk.virtualWrite(V0, String(latitude, 6));
    Blynk.virtualWrite(V1, String(longitude, 6));
//    myMap.location(move_index, latitude, longitude, "GPS_Location");
    
    //get speed
    gps_speed = gps.speed.kmph();
    Blynk.virtualWrite(V2, gps_speed);
       
    //get number of satellites
    no_of_satellites = gps.satellites.value();
    Blynk.virtualWrite(V3, no_of_satellites);

    // get the satellite orientation/direction
    satellite_orientation = TinyGPSPlus::cardinal(gps.course.value());
    Blynk.virtualWrite(V4, satellite_orientation);
      
    if (lastlatitude == 0.00 || lastlongitude == 0.00) {
      lastlatitude = latitude;
      lastlongitude = longitude;
      Serial.print("== 0.00 | ");

      Serial.print("La:  "); Serial.print(latitude,6);
      Serial.print(" |Lo: "); Serial.print(longitude,6);
      Serial.print(" |Sp:  "); Serial.print(gps_speed,3);
      Serial.print(" |Sa: "); Serial.print(no_of_satellites);
      Serial.print(" |Or: "); Serial.println(satellite_orientation);
    }

    float selisihlat = abs(latitude - lastlatitude);
    float selisihlong = abs(longitude - lastlongitude);
    float maxselisih = JarakMax / GarisBujur;
    
    if(selisihlat > maxselisih || selisihlong > maxselisih) {
      Serial.print("> Max Selisih | ");
      Serial.print("selisihlat:  "); Serial.print(selisihlat,6);
      Serial.print(" |selisihlong: "); Serial.print(selisihlong,6);
      Serial.print(" |maxselisih:  "); Serial.println(maxselisih,6);
      
      Serial.print("La:  "); Serial.print(latitude,6);
      Serial.print(" |Lo: "); Serial.print(longitude,6);
      Serial.print(" |Sp:  "); Serial.print(gps_speed,3);
      Serial.print(" |Sa: "); Serial.print(no_of_satellites);
      Serial.print(" |Or: "); Serial.println(satellite_orientation);
      
      EEPROM.write(EE_Status, 1);
      EEPROM.commit();
      delay(50);
      Status = EEPROM.read(EE_Status);
        
      if(!BuzzerON) BuzzerON = true;
    }
    lastlatitude = latitude;
    lastlongitude = longitude;
  }
}
