#include "FastLED.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_I2CDevice.h>

//--------------------------------------------------------
#define DEBUG           //Uncomment to activate debug

#define NUM_LEDS 88     //Nombre de led 88
#define DATA_PIN_LED 19 //Numéro du pin de données led 19
#define LUM 255         //Luminosité led [0;255]
#define Rp 10           //Intensité rouge feu position [0;255]
#define Rs 255          //Intensité rouge feu stop [0;255]
#define ClignoD_PIN 25  //Numéro du pin clignotant droit 25
#define ClignoG_PIN 27  //Numéro du pin clignotant gauche 27
#define Stop_PIN 32     //Numéro du pin feu stop 32
#define TriggerValue 30 //Seuil declenchement

CRGB leds[NUM_LEDS];
float vPow = 3.3;     //Tension ADC ref
float r1 = 30000;     //Résitance pont1 div
float r2 = 7500;      //Résitance pont2 div
float Vstop = 0;      //Tension courante Feu stop
float VclignoD = 0;   //Tension courante Clignotant droit
float VclignoG = 0;   //Tension courante Clignotant gauche
bool strob = 0;       
int etat = 0;         //Etat courant des led
int R = 0;            //Valeur courante rouge led

//----------------------------------------------------

Adafruit_BME280 bme;
unsigned long delayTime;
float temperature = 0, humidity = 0, pressure = 0;

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
//Configuration du timer pour envoie des mesures périodique
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
  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println("\n");
  #endif
  //----------------------------------------------------BME280
  bool status;
  status = bme.begin(0x76);
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    //while (1);
  }
  //----------------------LEDS---------------------------
  FastLED.addLeds<WS2812B, DATA_PIN_LED, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(LUM);
  //-------------Initialisation LED----------------------
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

//Mesure toute les x secondes (selon param timer)
  if (flagMeas == 1)
  {
    temperature = bme.readTemperature() * 100;
    humidity = bme.readHumidity() * 100;
    pressure = bme.readPressure() * 100;
    flagMeas = 0;
    
    #ifdef DEBUG
    Serial.printf("Pressure: %.2fhPa Temperature: %.2f°C Humidity: %.2f%% \n",pressure/10 ,temperature/100 ,humidity/100);
    //Serial.printf("VclignoG: %f VclignoD: %f Vstop: %f \n",VclignoG,VclignoD,Vstop);
    #endif
  }

  #ifdef DEBUG
  //Serial.printf("Pressure: $%.2fhPa Temperature: %d°C Humidity: %d%% \n",pressure / 10,temperature / 100,humidity / 100);
  //Serial.printf("VclignoG: $%d %d %d; VclignoD:  Vstop:  \n",(int)VclignoG,(int)VclignoD,(int)Vstop);
  //delay(10);
  #endif
  //Lecture des tensions
  float v0 = (analogRead(Stop_PIN) * vPow) / 1024.0;
  Vstop = v0 / (r2 / (r1 + r2));
  float v1 = (analogRead(ClignoD_PIN) * vPow) / 1024.0;
  VclignoD = v1 / (r2 / (r1 + r2));
  float v2 = (analogRead(ClignoG_PIN) * vPow) / 1024.0;
  VclignoG = v2 / (r2 / (r1 + r2));
  //Mise à jour de R
  R = (Vstop > TriggerValue) ? Rs : Rp;
  
  if (strob == 1)
  {
    etat = 6; //Strob
  }

  else if (VclignoD > TriggerValue and VclignoG > TriggerValue)
  {
    etat = 1; //Warning
  }

  else if (VclignoD > TriggerValue and VclignoG < TriggerValue)
  {
    etat = 2; //Clignotant D
  }

  else if (VclignoG > TriggerValue and VclignoD < TriggerValue)
  {
    etat = 3; //Clignotants G
  }

  else if (Vstop > TriggerValue)
  {
    etat = 4; //Feu STOP Allumage
  }

  else if (Vstop < TriggerValue)
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
