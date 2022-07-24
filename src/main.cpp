#include <FastLED.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_I2CDevice.h>

//--------------------------------------------------------
#define DEBUG // Uncomment to activate debug

#define NUM_LEDS 88                // Nombre de led 88                      LEDS
#define DATA_PIN_LED 19            // Numéro du pin de données led 19       PIN
#define LUM 50                     // Luminosité led                      [0;255]
#define Rp 10                      // Intensité rouge feu position        [0;255]
#define Rs 255                     // Intensité rouge feu stop            [0;255]
#define ClignoD_PIN 25             // Numéro du pin clignotant droit 25     PIN
#define ClignoG_PIN 27             // Numéro du pin clignotant gauche 27    PIN
#define Stop_PIN 32                // Numéro du pin feu stop 32             PIN
#define TriggerValue 12            // Seuil declenchement                   (V)
#define TurnSignalPeriode_ms 100 // Période des clignotant                (ms)
//--Orange
#define Ogreen 40

CRGB leds[NUM_LEDS];
float vPow = 3.3;     // Tension ADC ref                     (V)
float r1 = 30000;     // Résitance pont1 div                 (Ohms)
float r2 = 7500;      // Résitance pont2 div                 (Ohms)
float v0, v1, v2 = 0; // Tension mesurée par l'ADC 12 bits   (V)
float Vstop = 0;      // Tension courante Feu stop           (V)
float VclignoD = 0;   // Tension courante Clignotant droit   (V)
float VclignoG = 0;   // Tension courante Clignotant gauche  (V)

bool strob = 0;
int etat = 0; // Etat courant des led
int R = 0;    // Valeur courante rouge led
int TurnSignalDelay = TurnSignalPeriode_ms / 88;
//----------------------------------------------------

Adafruit_BME280 bme;
unsigned long delayTime;
float temperature = 0, humidity = 0, pressure = 0;

//Analog reading refresh
void input_refresh()
{
  // Lecture des tensions
  v0 = (analogRead(Stop_PIN) * vPow) / 4095.0;
  Vstop = v0 / (r2 / (r1 + r2));
  v1 = (analogRead(ClignoD_PIN) * vPow) / 4095.0;
  VclignoD = v1 / (r2 / (r1 + r2));
  v2 = (analogRead(ClignoG_PIN) * vPow) / 4095.0;
  VclignoG = v2 / (r2 / (r1 + r2));
  // Mise à jour de R
  R = (Vstop > TriggerValue) ? Rs : Rp;
}
//----------------------Timer-----------------------
// Configuration du timer pour envoie des mesures périodique
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
    // while (1);
  }
  //----------------------LEDS---------------------------
  FastLED.addLeds<WS2812B, DATA_PIN_LED, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(LUM);
  //-------------Initialisation LED----------------------
  for (int i = 1; i <= (NUM_LEDS / 2); i++)
  {
    leds[i - 1].setRGB(0, (5 * i) + 35, 0);
    leds[NUM_LEDS - i].setRGB(0, (5 * i) + 35, 0);
    FastLED.show();
    delay(20);
  }
  // Fade out to Rp
  for (int i = 0; i <= 255 - Rp; i++)
  {
    fill_solid(leds, NUM_LEDS, CRGB(0, 255 - i, 0));
    FastLED.show();
    delay(5);
  }
}

void loop()
{

  // Mesure toute les secondes (selon param timer)
  if (flagMeas == 1)
  {
    flagMeas = 0;
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0; // Convert pressure hPa

#ifdef DEBUG
    Serial.printf("\n  _____Measurements:_____  \nPressure: %.2fhPa Temperature: %.2f°C Humidity: %.2f%% \n", pressure, temperature, humidity);
    Serial.printf("VclignoG,VclignoD,Vstop[V]: $%d %d %d;  \n", (int)VclignoG, (int)VclignoD, (int)Vstop);
    Serial.printf("v0,1,2[V]: $%f %f %f; \n", v0, v1, v2);
#endif
  }

  input_refresh();

  if (strob == 1)
  {
    etat = 6; // Strob
  }

  else if (VclignoD > TriggerValue && VclignoG > TriggerValue)
  {
    etat = 1; // Warning
  }

  else if (VclignoD > TriggerValue && VclignoG < TriggerValue)
  {
    etat = 2; // Clignotant D
  }

  else if (VclignoG > TriggerValue && VclignoD < TriggerValue)
  {
    etat = 3; // Clignotants G
  }

  else if (Vstop > TriggerValue)
  {
    etat = 4; // Feu STOP Allumage
  }

  else if (Vstop < TriggerValue)
  {
    etat = 5; // Feu STOP Extinction
  }

  switch (etat)
  {
  case 0:
    // Feux de positions
    fill_solid(leds, NUM_LEDS, CRGB(0, Rp, 0));
    FastLED.show();
    break;
  case 1:
  {
    // Warning
    int lastR = R;
    // Allumage
    for (int i = 0; i <= (NUM_LEDS / 2); i++)
    {
      leds[0 + i] = CRGB(Ogreen, 255, 0);
      leds[NUM_LEDS - i] = CRGB(Ogreen, 255, 0);
      FastLED.show();
      delay(TurnSignalDelay);
      // Maj stop light
      input_refresh();
      if (R != lastR)
      {
        // Stop 1er moitier
        for (int x = i + 1; x < NUM_LEDS / 2; x++)
        {
          leds[x] = CRGB(0, R, 0);
        }
        // Stop 2eme moitier
        for (int x = (NUM_LEDS / 2); x < NUM_LEDS - i; x++)
        {
          leds[x] = CRGB(0, R, 0);
        }
        lastR = R;
      }
      if(VclignoD < TriggerValue || VclignoG < TriggerValue)
      {
        fill_solid(leds,NUM_LEDS, CRGB(0, R, 0));
        FastLED.show();
        break;
      }
    }
    // Extinction
    for (int i = 0; i <= (NUM_LEDS / 2); i++)
    {
      leds[0 + i] = CRGB(0, R, 0);
      leds[NUM_LEDS - i] = CRGB(0, R, 0);
      FastLED.show();
      delay(TurnSignalDelay);
      // Maj stop light
      input_refresh();
      if (R != lastR)
      {
        // Stop 1er moitier
        for (int x = 0; x < i + 1; x++)
        {
          leds[x] = CRGB(0, R, 0);
        }
        // Stop 2eme moitier
        for (int x = 0; x < i + 1; x++)
        {
          leds[NUM_LEDS - x] = CRGB(0, R, 0);
        }
        lastR = R;
      }
      if(VclignoD < TriggerValue || VclignoG < TriggerValue)
      {
        fill_solid(leds,NUM_LEDS, CRGB(0, R, 0));
        FastLED.show();
        break;
      }
    }
  }
  break;
  case 2:
  {
    // Clignotants 1er moitier
    int lastR = R;
    fill_solid(leds, NUM_LEDS, CRGB(0, R, 0));
    // Allumage
    for (int i = 0; i <= (NUM_LEDS / 2) - 1; i++)
    {
      leds[i] = CRGB(Ogreen, 255, 0);
      FastLED.show();
      delay(TurnSignalDelay);
      // Maj stop light
      input_refresh();
      if (R != lastR)
      {
        // Stop 1er moitier
        for (int x = i + 1; x < NUM_LEDS / 2; x++)
        {
          leds[x] = CRGB(0, R, 0);
        }
        // Stop 2eme moitier
        for (int x = (NUM_LEDS / 2); x < NUM_LEDS; x++)
        {
          leds[x] = CRGB(0, R, 0);
        }
        lastR = R;
      }
      if(VclignoG > TriggerValue)
      {
        fill_solid(leds,NUM_LEDS, CRGB(0, R, 0));
        FastLED.show();
        break;
      }
    }
    // Extinction
    for (int i = 0; i <= (NUM_LEDS / 2); i++)
    {
      leds[i] = CRGB(0, R, 0);
      FastLED.show();
      delay(TurnSignalDelay);
      // Maj stop light
      input_refresh();
      if (R != lastR)
      {
        // Stop 1er moitier
        for (int x = 0; x < i + 1; x++)
        {
          leds[x] = CRGB(0, R, 0);
        }
        // Stop 2eme moitier
        for (int x = (NUM_LEDS / 2); x < NUM_LEDS; x++)
        {
          leds[x] = CRGB(0, R, 0);
        }
        lastR = R;
      }
      if(VclignoG > TriggerValue)
      {
        fill_solid(leds,NUM_LEDS, CRGB(0, R, 0));
        FastLED.show();
        break;
      }
    }

    break;
  }
  case 3:
  {
    // Clignotants 2em moitier
    int lastR = R;
    fill_solid(leds, NUM_LEDS, CRGB(0, R, 0));
    // Allumage
    for (int i = 0; i <= (NUM_LEDS / 2); i++)
    {
      leds[NUM_LEDS - i] = CRGB(Ogreen, 255, 0);
      FastLED.show();
      delay(TurnSignalDelay);
      // Maj stop light
      input_refresh();
      if (R != lastR)
      {
        // Stop 2eme moitier
        for (int x = (NUM_LEDS / 2); x < NUM_LEDS - i; x++)
        {
          leds[x] = CRGB(0, R, 0);
        }
        // Stop 1ere moitier
        for (int x = 0; x < NUM_LEDS / 2; x++)
        {
          leds[x] = CRGB(0, R, 0);
        }
        lastR = R;
      }
      if(VclignoD > TriggerValue)
      {
        fill_solid(leds,NUM_LEDS, CRGB(0, R, 0));
        FastLED.show();
        break;
      }
    }
    // Extinction
    for (int i = 0; i <= (NUM_LEDS / 2); i++)
    {
      leds[NUM_LEDS - i] = CRGB(0, R, 0);
      FastLED.show();
      delay(TurnSignalDelay);
      // Maj stop light
      input_refresh();
      if (R != lastR)
      {
        // Stop 2eme moitier
        for (int x = 0; x < i + 1; x++)
        {
          leds[NUM_LEDS - x] = CRGB(0, R, 0);
        }
        // Stop 1er moitier
        for (int x = 0; x < NUM_LEDS / 2; x++)
        {
          leds[x] = CRGB(0, R, 0);
        }
        lastR = R;
      }
      if(VclignoD > TriggerValue)
      {
        fill_solid(leds,NUM_LEDS, CRGB(0, R, 0));
        FastLED.show();
        break;
      }
    }
    break;
  }
  case 4:
    // Feu STOP Allumage
    fill_solid(leds, NUM_LEDS, CRGB(0, R, 0));
    FastLED.show();
    break;
  case 5:
    // Feu STOP Extinction
    fill_solid(leds, NUM_LEDS, CRGB(0, R, 0));
    FastLED.show();
    break;
  case 6:
    // Strob
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
