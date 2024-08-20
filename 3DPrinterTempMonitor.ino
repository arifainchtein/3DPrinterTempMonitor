
#include <FastLED.h>
#include <Timer.h>
#include <TM1637Display.h>

#include "OneWire.h"
#include "DallasTemperature.h"

#include <PCF8563TimeManager.h>
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define LED_PIN 19
#define LED_PIN 19
#define NUM_LEDS 8
CRGB leds[NUM_LEDS];
#define SDA_I2C 21

#define SCL_I2C 22

#define RELAY 32
#define UI_CLK 23
#define UI1_DAT 26
#define UI2_DAT 25

//
// for Pancho build 8 and below is 18
// from build 9 onwards is 4
//
#define RTC_CLK_OUT 4

#define SENSOR1 5
OneWire oneWire(SENSOR1);
DallasTemperature tempSensor(&oneWire);
TM1637Display display1(UI_CLK, UI1_DAT);
TM1637Display display2(UI_CLK, UI2_DAT);
volatile bool clockTicked = false;
Timer sampleTimer(60);
uint8_t targetTemperature=27;
float initialTemperature;

PCF8563TimeManager timeManager(Serial);

void IRAM_ATTR clockTick() {
  portENTER_CRITICAL_ISR(&mux);
  clockTicked = true;
  portEXIT_CRITICAL_ISR(&mux);
}


void setup() {
   Serial.begin(115200);
  // put your setup code here, to run once:
FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
pinMode(RTC_CLK_OUT, INPUT_PULLUP);  // set up interrupt pin
  digitalWrite(RTC_CLK_OUT, HIGH);     // turn on pullup resistors
  // attach interrupt to set_tick_tock callback on rising edge of INT0
  attachInterrupt(digitalPinToInterrupt(RTC_CLK_OUT), clockTick, RISING);
  timeManager.start();
  timeManager.PCF8563osc1Hz();
  
 tempSensor.begin();
  uint8_t address[8];
  tempSensor.getAddress(address, 0);
  String serialNumber;
 for (  uint8_t i = 0; i < 8; i++){
      serialNumber += String(address[i], HEX);
 }
 Serial.print("sn=");
 Serial.println(serialNumber);
 sampleTimer.start();
  tempSensor.requestTemperatures(); // Send the command to get temperatures
   initialTemperature = (uint8_t)tempSensor.getTempCByIndex(0);
   Serial.print("initialTemperature=");
   Serial.println(initialTemperature);
  
   display1.setBrightness(0x0f);
   display2.setBrightness(0x0f);
  // pinMode(RELAY, OUTPUT);
   
   Serial.println("done");
}

void loop() {
  // put your main code here, to run repeatedly:
 
  tempSensor.requestTemperatures(); // Send the command to get temperatures
  
   float temperature = (uint8_t)tempSensor.getTempCByIndex(0);
   Serial.print("temperature=");
   Serial.println(temperature);
   
  display1.showNumberDec(temperature, false);
  if(temperature<targetTemperature){
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(255, 0, 0);
    }
    FastLED.show();
    
     
   // digitalWrite(RELAY, HIGH); // Turn the relay on
  }else{
     for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(0, 255, 0);
    }
   // digitalWrite(RELAY, LOW);
    FastLED.show();
  }

   if (clockTicked) {
      portENTER_CRITICAL(&mux);
      clockTicked = false;
      portEXIT_CRITICAL(&mux);
      sampleTimer.tick();
   }

   if(sampleTimer.status()){
     sampleTimer.reset();
     float increaseTemperature = temperature-initialTemperature;
     float durationMinutes = millis()/60000;
     float rateOfIncrease = increaseTemperature/durationMinutes;
     float remainingMinutes=(targetTemperature-temperature)*rateOfIncrease;
     display2.showNumberDec(remainingMinutes, false);
   }
}