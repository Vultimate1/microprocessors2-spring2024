#include <Wire.h>                   // for I2C communication
#include <LiquidCrystal_I2C.h>      // for LCD
#include <RTClib.h>                 // for RTC
#include <arduinoFTT.h>

const int motor = 5;
const double speed_settings = {0, 0.5, 0.75, 1}; //settings for the motor's speed

//I2C address to communicate with DS3231 = 0x68
//I2C bus address = 0x57
RTC_DS3231 rtc;
const int analogSound = 3;
const int digitalSound = 4;
int frequencies = {};
int numsamples = 0;
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, samplingFrequency, true);
const uint16_t samples = 64; // number of samples for FFT when finding max frequency
int A4 = 440
int C4 = 262 
double error = 0.02 //to catch any notes near C4 or A4
void setup() {
  // put your setup code here, to run once:
  pinMode(motor, OUTPUT);
  pinMode(analogSound, INPUT);
  pinMode(digitalSound, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int analog = analogRead(analogSound);
  int digital = digitalRead(digitalSound); //what to convert and use for sampling, on a scale of 0 to 5 V
  /* 1. Convert digital value to sound, via a conversion factor (presumably (A4-C4)/5 for the steps.*/
  int sound = digital*(A4-C4)/5 + 267; //default value is 267 Hz, so default speed is 0
  samples[numsamples] = sound;
  for (int i=0; i<samples; i++) {
     
  }
  /* 2. Add sound to array for sampling, to use for FFT. */
  /* 3. This will have to be in timer interrupt to enable this to be continuously called.  */ 
  /* 4. Periodically check the maximum frequency by FFT.majorPeak() function. */
  /* 5. If maximum frequency is C4, then get current speed of motor, and if it matches a speed setting 
   * in the motor speed setting array (with an error of 2%), then increase it to the next highest setting.
   *    If the maximum frequency is more or less A4 (with an error of 2%), decrease it to the next lowest setting.
   */
}
