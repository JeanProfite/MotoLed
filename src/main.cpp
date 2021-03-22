#include "FastLED.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define NUM_LEDS 60     //Nombre de led 88
#define DATA_PIN_LED 19 //Numéro du pin de données led
#define LUM 255         //Luminosité led [0;255]
#define Rp 10           //Intensité rouge feu position [0;255]
#define Rs 255          //Intensité rouge feu stop [0;255]
#define ClignoD_PIN 25
#define ClignoG_PIN 33
#define Stop_PIN 32

CRGB leds[NUM_LEDS];
float vPow = 4.7;     //Tension ADC ref
float r1 = 30000;     //Résitance pont1 div
float r2 = 7500;      //Résitance pont2 div
float Vstop = 0;      //Tension courante Feu stop
float VclignoD = 0;   //Tension courante Clignotant droit
float VclignoG = 0;   //Tension courante Clignotant gauche
bool strob = 0;
int etat = 0;     //Etat courant des led
int R = 0;        //Valeur courante rouge led

//----------------------------------------------------BLE-----------------------------------
bool deviceConnected = false;

void BLETransfer(int16_t);

#define ServiceUUID "c9a110ba-7f3e-4165-847c-17f92e4ee6b3"
#define DescriptorUUID BLEUUID((uint16_t)0x2901)

#define temperatureCharacteristicUUID BLEUUID((uint16_t)0x2A6E)
#define humidityCharacteristicUUID BLEUUID((uint16_t)0x2A6F)
#define pressureCharacteristicUUID BLEUUID((uint16_t)0x2A6D)
// --------------------------Characteristic definition----------------------------------------
BLECharacteristic temperatureCharacteristic(
    temperatureCharacteristicUUID,
    BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic humidityCharacteristic(
    humidityCharacteristicUUID,
    BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic pressureCharacteristic(
    pressureCharacteristicUUID,
    BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor tempDescriptor(DescriptorUUID);
BLEDescriptor humDescriptor(DescriptorUUID);
BLEDescriptor pressureDescriptor(DescriptorUUID);

//----------------------------BLE Callbacks------------------------------------

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("Device connected");
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

//----------------------------------------------------

Adafruit_BME280 bme;
unsigned long delayTime;
int temperature = 0, humidity = 0, pressure = 0;

void fill_half_grad(int Half, uint8_t Red, uint8_t Green, uint8_t Blue)
{
  int f = 0;
  for (int i = 0; i <= (NUM_LEDS / 2); i++)
  {
    f = (Half == 1) ? -1 + i : (NUM_LEDS - i);
    leds[f] = CRGB(Green, Red, Blue);
    FastLED.show();
    delay(5);
  }
}
//----------------------Timer-----------------------
volatile bool flagMeas;
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTime()
{
  portENTER_CRITICAL_ISR(&timerMux);
  flagMeas = !flagMeas;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup()
{
  //----------------------------------------------------Timer
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTime, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);

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

  pressureDescriptor.setValue("pressure ATM (Pa)");
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
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
  //----------------------------------------------------Leds
  FastLED.addLeds<WS2812B, DATA_PIN_LED, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(LUM);
  //-------------Démarage led
  for (int i = 0; i <= (NUM_LEDS / 2); i++)
  {
    leds[i].setRGB(0, 3 * i, 0);
    leds[NUM_LEDS - i].setRGB(0, 3 * i, 0);
    FastLED.show();
    delay(20);
  }

  for (int i = 0; i <= 255 - Rp; i++)
  {
    fill_solid(leds, NUM_LEDS, CRGB(0, 255 - i, 0));
    FastLED.show();
    delay(5);
  }
}

void loop()
{

//Envoie des mesures des capteurs si un client est connecté toute les x secondes (selon param timer)
  if (deviceConnected && flagMeas == 1)
  {
    temperature = bme.readTemperature() * 100;
    humidity = bme.readHumidity() * 100;
    pressure = bme.readPressure() * 100;

    Serial.print("Pressure(hPa): ");
    Serial.println(pressure / 10);
    Serial.print("Temperature(°C): ");
    Serial.println(temperature / 100);
    Serial.print("Humidity(%): ");
    Serial.println(humidity / 100);
    Serial.print("VclignoG: ");
    Serial.println(VclignoG);
    Serial.print("VclignoD: ");
    Serial.println(VclignoD);
    Serial.print("Vstop: ");
    Serial.println(Vstop);
    Serial.println("");

    /* Set the value */
    temperatureCharacteristic.setValue((uint8_t *)&temperature, 2); // This is a value of a single byte
    temperatureCharacteristic.notify();                             // Notify the client of a change
    humidityCharacteristic.setValue((uint8_t *)&humidity, 2);       // This is a value of a single byte
    humidityCharacteristic.notify();                                // Notify the client of a change
    pressureCharacteristic.setValue((uint8_t *)&pressure, 4);       // This is a value of a single byte
    pressureCharacteristic.notify();                                // Notify the client of a change
    flagMeas = 0;
  }

  //Lecture des tensions
  float v0 = (analogRead(Stop_PIN) * vPow) / 1024.0;
  Vstop = v0 / (r2 / (r1 + r2));
  float v1 = (analogRead(ClignoD_PIN) * vPow) / 1024.0;
  VclignoD = v1 / (r2 / (r1 + r2));
  float v2 = (analogRead(ClignoG_PIN) * vPow) / 1024.0;
  VclignoG = v2 / (r2 / (r1 + r2));
  //Mise à jour de R
  R = (Vstop > 8) ? Rs : Rp;
  if (strob == 1)
  {
    etat = 6; //Strob
  }
  else if (VclignoD > 8 and VclignoG > 8)
  {
    etat = 1; //Warning
  }
  else if (VclignoD > 8 and VclignoG < 8)
  {
    etat = 2; //Clignotant D
  }
  else if (VclignoG > 8 and VclignoD < 8)
  {
    etat = 3; //Clignotants G
  }
  else if (Vstop > 8)
  {
    etat = 4; //Feu STOP Allumage
  }
  else if (Vstop < 8)
  {
    etat = 5; //Feu STOP Extinction
  }

  switch (etat)
  {
  case 0:
    //Feux de positions
    fill_solid(leds, NUM_LEDS, CRGB(0, Rp, 0));
    FastLED.show();
    break;
  case 1:
    //Warning
    for (int i = 0; i <= (NUM_LEDS / 2); i++)
    {
      leds[0 + i] = CRGB(69, 255, 0);
      leds[NUM_LEDS - i] = CRGB(69, 255, 0);
      FastLED.show();
      delay(5);
    }
    for (int i = 0; i <= (NUM_LEDS / 2); i++)
    {
      leds[0 + i] = CRGB(0, R, 0);
      leds[NUM_LEDS - i] = CRGB(0, R, 0);
      FastLED.show();
      delay(5);
    }
    break;
  case 2:
    //Clignotants 1er moitier
    fill_solid(leds, NUM_LEDS, CRGB(0, R, 0));
    fill_half_grad(1, 255, 69, 0);
    fill_half_grad(1, R, 0, 0);
    break;
  case 3:
    //Clignotants 2em moitier
    fill_solid(leds, NUM_LEDS, CRGB(0, R, 0));
    fill_half_grad(2, 255, 69, 0);
    fill_half_grad(2, R, 0, 0);
    break;
  case 4:
    //Feu STOP Allumage
    fill_solid(leds, NUM_LEDS, CRGB(0, R, 0));
    FastLED.show();
    break;
  case 5:
    //Feu STOP Extinction
    fill_solid(leds, NUM_LEDS, CRGB(0, R, 0));
    FastLED.show();
    break;
  case 6:
    //Strob
    for (int i = 0; i <= (NUM_LEDS / 2); i++)
    {
      leds[NUM_LEDS - i] = CRGB(0, 0, 255);
    }
    FastLED.show();
    delay(100);
    for (int i = 0; i <= (NUM_LEDS / 2); i++)
    {
      leds[NUM_LEDS - i] = CRGB(0, 0, 0);
    }
    FastLED.show();
    delay(50);
    fill_solid(leds, NUM_LEDS / 2, CRGB(0, 0, 255));
    FastLED.show();
    delay(100);
    fill_solid(leds, NUM_LEDS / 2, CRGB(0, 0, 0));
    FastLED.show();
    delay(50);
    break;
  }
}
