#include "FastLED.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
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

//----------------------------------------------------BLE
int16_t value = 0;  //the set value function only accepts unsigned 8 bit integers
int16_t value2 = 0;
int16_t value3 = 0;

bool deviceConnected = false;

void BLETransfer(int16_t);

#define ServiceUUID "c9a110ba-7f3e-4165-847c-17f92e4ee6b3"
  #define DescriptorUUID BLEUUID((uint16_t)0x2901)
  
  #define temperatureCharacteristicUUID BLEUUID((uint16_t)0x2A6E)
  #define humidityCharacteristicUUID BLEUUID((uint16_t)0x2A6F)
  #define pressureCharacteristicUUID BLEUUID((uint16_t)0x2A6D)
//-----------------------------------------------------------------------
BLECharacteristic temperatureCharacteristic(
  temperatureCharacteristicUUID, 
  BLECharacteristic::PROPERTY_READ | 
  BLECharacteristic::PROPERTY_NOTIFY
);
BLECharacteristic humidityCharacteristic(
  humidityCharacteristicUUID, 
  BLECharacteristic::PROPERTY_READ | 
  BLECharacteristic::PROPERTY_NOTIFY
);
BLECharacteristic pressureCharacteristic(
  pressureCharacteristicUUID, 
  BLECharacteristic::PROPERTY_READ | 
  BLECharacteristic::PROPERTY_NOTIFY
);
BLEDescriptor tempDescriptor(DescriptorUUID);
BLEDescriptor humDescriptor(DescriptorUUID);
BLEDescriptor pressureDescriptor(DescriptorUUID);

//-----------------------------------------------------------------------

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

//----------------------------------------------------


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
//----------------------------------------------------BLE
 // Create the BLE Device
  BLEDevice::init("ESP");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(ServiceUUID);

  // Create a BLE Characteristic
  //-----------------------------------------------------------------------
  pService->addCharacteristic(&temperatureCharacteristic);
  pService->addCharacteristic(&humidityCharacteristic);
  pService->addCharacteristic(&pressureCharacteristic);

  //-----------------------------------------------------------------------
  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  //-----------------------------------------------------------------------
  tempDescriptor.setValue("Temperature -40-60°C");
  temperatureCharacteristic.addDescriptor(&tempDescriptor);
  temperatureCharacteristic.addDescriptor(new BLE2902());

  humDescriptor.setValue("Humidity 0-100%");
  humidityCharacteristic.addDescriptor(&humDescriptor);
  humidityCharacteristic.addDescriptor(new BLE2902());

  pressureDescriptor.setValue("Pression ATM (Pa)");
  pressureCharacteristic.addDescriptor(&pressureDescriptor);
  pressureCharacteristic.addDescriptor(new BLE2902());
  //-----------------------------------------------------------------------
  pServer->getAdvertising()->addServiceUUID(ServiceUUID);

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();

  Serial.println("Waiting for a Client to connect...");
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
value = temper;
value2 = hum;
value3 = press;

if (deviceConnected) {
/* Set the value */
  temperatureCharacteristic.setValue((uint8_t*)&value,2);  // This is a value of a single byte
  temperatureCharacteristic.notify();  // Notify the client of a change
  humidityCharacteristic.setValue((uint8_t*)&value2,2);  // This is a value of a single byte
  humidityCharacteristic.notify();  // Notify the client of a change
  pressureCharacteristic.setValue((uint8_t*)&value3,2);  // This is a value of a single byte
  pressureCharacteristic.notify();  // Notify the client of a change
}

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


