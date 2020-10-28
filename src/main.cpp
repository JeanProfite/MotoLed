#include "FastLED.h"
#include <Wifi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#define NUM_LEDS 60 //Nombre de leds 88
#define DATA_PIN 19  //Numéro du pin de données
#define LUM 255 //Luminosité [0;255]
#define Rp 10 //Rouge feu position [0;255]
#define ClignoD_PIN 34
#define ClignoG_PIN 35
#define Stop_PIN 32

CRGB leds[NUM_LEDS];
float vPow = 4.7;
float r1 = 30000;
float r2 = 7500;
float Vstop =0;
float VclignoD =0;
float VclignoG =0;
bool strob=0;
const char* ssid = "NC700S";
const char* password = "password";

AsyncWebServer server(80);

Adafruit_BME280 bme;
unsigned long delayTime;
float temper=0,hum=0,press=0;

void majBME() {
  temper=bme.readTemperature();
  hum=bme.readHumidity();
  press=bme.readPressure() / 100.0F;

} 

void setup() { 
//----------------------------------------------------Serial
Serial.begin(115200);
Serial.println("\n");
//----------------------------------------------------SPIFFS
if(!SPIFFS.begin()){
  Serial.println("Erreur SPIFFS...");
  return;
}

File root = SPIFFS.open("/");
File file = root.openNextFile();

while(file)
{
   Serial.print("File: ");
   Serial.println(file.name());
   file.close();
   file = root.openNextFile();
 }
//----------------------------------------------------WIFI
Serial.println("\n");
Serial.println("Creation du point d'acces...");
WiFi.softAP(ssid,password);
Serial.print("Adresse IP: ");
Serial.println(WiFi.softAPIP());
//----------------------------------------------------SERVER
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/w3.css", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/w3.css", "text/css");
  });

  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/script.js", "text/javascript");
  });

  server.on("/lireTemperature", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    float val = temper;
    String temperature = String(val);
    request->send(200, "text/plain", temperature);
  });

    server.on("/lireHumidite", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    float val = hum;
    String Shum = String(val);
    request->send(200, "text/plain", Shum);
  });

      server.on("/lirePression", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    float val = press;
    String Spress = String(val);
    request->send(200, "text/plain", Spress);
  });

  server.on("/lireStop", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    float val = Vstop;
    String SVstop = String(val);
    request->send(200, "text/plain", SVstop);
  });

  server.on("/lireClignoG", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    float val = VclignoG;
    String SVclignoG = String(val);
    request->send(200, "text/plain", SVclignoG);
  });

    server.on("/lireClignoD", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    float val = VclignoD;
    String SVclignoD = String(val);
    request->send(200, "text/plain", SVclignoD);
  });

  server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    strob=1;
    request->send(200);
  });

  server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    strob=0;
    request->send(200);
  });

  server.begin();
  Serial.println("Serveur actif!");
//----------------------------------------------------BME280
bool status;
status = bme.begin(0x76); 
if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
//----------------------------------------------------Leds
FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
FastLED.setBrightness( LUM );
//-------------Démarage led
for (int i=0; i <= (NUM_LEDS/2); i++){ 
  leds[i].setRGB( 0, 3*i,0);
  leds[NUM_LEDS-i].setRGB( 0,3*i , 0);
  FastLED.show();
   delay(20);
  }
 
for (int i=0; i <= 255-Rp; i++){ 
  fill_solid(leds,NUM_LEDS, CRGB(0, 255-i , 0));
      FastLED.show();
  delay(5);
  }

}

void loop() {
  majBME();
  //Lecture des tensions
   float v0 = (analogRead(Stop_PIN) * vPow) / 1024.0;
  Vstop = v0 / (r2 / (r1 + r2));
   float v1 = (analogRead(ClignoD_PIN) * vPow) / 1024.0;
    VclignoD = v1 / (r2 / (r1 + r2));
   float v2 = (analogRead(ClignoG_PIN) * vPow) / 1024.0;
    VclignoG = v2 / (r2 / (r1 + r2));
//Feux de positions
fill_solid(leds,NUM_LEDS, CRGB( 0, Rp, 0));
        FastLED.show();
        
//Warning
if (VclignoD>8 and VclignoG>8) { 
   for (int i=0; i <= (NUM_LEDS/2); i++){ 
    leds[0+i] = CRGB( 69, 255, 0);
    leds[NUM_LEDS-i] = CRGB( 69, 255, 0);
    FastLED.show();
    delay(5);
    }
  for (int i=0; i <= (NUM_LEDS/2); i++){ 
    leds[0+i] = CRGB( 0, Rp, 0);
    leds[NUM_LEDS-i] = CRGB( 0, Rp, 0);
    FastLED.show();
    delay(5);
    }
} 


//Clignotants 1er moitier
if (VclignoD>8 and VclignoG<8) { 
for (int i=0; i <= (NUM_LEDS/2)-1; i++){ 
    leds[0+i] = CRGB( 69, 255, 0);
    FastLED.show();
    delay(5);
    }
  for (int i=0; i <= (NUM_LEDS/2)-1; i++){ 
    leds[0+i] = CRGB( 0, Rp, 0);
    FastLED.show();
    delay(5);
    }
} 
  
//Clignotants 2em moitier
if (VclignoG>8 and VclignoD<8) { 
  for (int i=0; i <= (NUM_LEDS/2); i++){ 
    leds[NUM_LEDS-i] = CRGB( 69, 255, 0);
    FastLED.show();
    delay(5);
    }
  for (int i=0; i <= (NUM_LEDS/2); i++){ 
    leds[NUM_LEDS-i] = CRGB( 0, Rp, 0);
    FastLED.show();
    delay(5);
    }
} 

//Feu STOP Allumage
if (Vstop>8) {
  fill_solid(leds,NUM_LEDS, CRGB( 0, 255, 0));
      FastLED.show();
}
  
//Feu STOP Extinction
  if(Vstop<8){
    fill_solid(leds,NUM_LEDS, CRGB( 0, Rp, 0));
        FastLED.show();
        }

//strobe
if (strob==1) { 
    for (int p=0; p <= (NUM_LEDS/2); p++){ 
    leds[NUM_LEDS-p] = CRGB( 0, 0, 255);
    }
    FastLED.show();
    delay(100);
    for (int d=0; d <= (NUM_LEDS/2); d++){ 
    leds[NUM_LEDS-d] = CRGB( 0, 0, 0);
    }
    FastLED.show();
    delay(50);
    fill_solid(leds,NUM_LEDS/2, CRGB( 0, 0, 255));
    FastLED.show();
    delay(100);
     fill_solid(leds,NUM_LEDS/2, CRGB( 0, 0, 0));
    FastLED.show();
    delay(50);
}
}


