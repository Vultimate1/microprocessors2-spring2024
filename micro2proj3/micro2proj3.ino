#include <LiquidCrystal.h>

//www.elegoo.com
//2016.12.9

/*
  LiquidCrystal Library - Hello World

 Demonstrates the use of a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.

 This sketch prints "Hello World!" to the LCD
 and shows the time.

  The circuit:
 * LCD RS pin to digital pin 7
 * LCD Enable pin to digital pin 8
 * LCD D4 pin to digital pin 9
 * LCD D5 pin to digital pin 10
 * LCD D6 pin to digital pin 11
 * LCD D7 pin to digital pin 12
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

 Library originally added 18 Apr 2008
 by David A. Mellis
 Library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 Example added 9 Jul 2009
 by Tom Igoe
 Modified 22 Nov 2010
 by Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/LiquidCrystal
 */

// include the library code:
#include <LiquidCrystal.h>
#include <Wire.h>
#include <DS3231.h>
#include <arduinoFFT.h>


DS3231 clock;
RTCDateTime dt;

#define ENABLE 5
#define DIRA 3
#define DIRB 4


//button INter
int interruptPin = 2;
volatile byte state = LOW;
bool clock1 = true;

//constants for speed
const double speed_settings[] = {0, 0.5, 0.75, 1}; //settings for the motor's speed
int curr_state = 0;

boolean toggle1 = 0;
boolean timeFlag = false;
int rotation = 0; 
double total_speed = 0;
int start_Fan = 0;
double main_freq = 0;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

//Constants for sound and frequency sampling
const int analogSound = A0;
const int digitalSound = 53;
int frequencies = {};
int numsamples = 0;
unsigned long ms = 0;
const uint16_t samples = 128; // number of samples for FFT when finding max frequency
double sampleReal[samples];
double sampleImag[samples];
const double samplingFrequency = 1600;
unsigned int time_const = round(1000000/samplingFrequency);

//arduinoFFT<double> FFT = ArduinoFFT<double>(sampleReal, sampleImag, samples, samplingFrequency, true);
arduinoFFT FFT = arduinoFFT();

double a4 = 440;
double c4 = 262;
double error = 0.02; //to catch any notes near C4 or A4


void setup() {

  cli();//stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts


  //---set pin direction - DC MOTOR
  pinMode(ENABLE,OUTPUT);
  pinMode(DIRA,OUTPUT);
  pinMode(DIRB,OUTPUT);

  //Button Setup
  pinMode(interruptPin, INPUT_PULLUP);
  digitalWrite(interruptPin, LOW);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, RISING);

  //sound
  //pinMode(analogSound, INPUT);
  pinMode(digitalSound, INPUT);
  Serial.begin(115200);

  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Speed: ");
  clock.begin();

  // Set sketch compiling time
  clock.setDateTime(__DATE__, __TIME__);
}

// Timer1's interrupt service routing (ISR)
// The code in ISR will be executed every time timer1 interrupt occurs
// That is, the code will be called once every second
// TODO
//   you need to set a flag to trigger an evaluation of the conditions
//   in your state machine, then potentially transit to next state
//
ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles
  timeFlag = true;
}

void loop() {
  //Set ms to current time
  ms = micros();
  for (int i=0; i<samples; i++) {
      sampleReal[i] = (double)analogRead(analogSound);  
      sampleImag[i] = 0;
      while(micros() < ms + time_const){

      }
      ms += time_const;
      
    }
    FFT.Windowing(sampleReal, samples, FFTWindow::Hamming, FFTDirection::Forward);
    FFT.Compute(sampleReal, sampleImag, samples, FFTDirection::Forward); /* Compute FFT */
    FFT.ComplexToMagnitude(sampleReal,  sampleImag, samples); /* Compute magnitudes */  
    Serial.print("Main frequency: ");
    main_freq = FFT.MajorPeak(sampleReal, samples, samplingFrequency); 
    Serial.println(main_freq); 
    //for (int i=0; i<samples/2; i++) {
      // Serial.println(sampleReal[i]);


    if(rotation == 1)
    {
      analogWrite(ENABLE,(255 * speed_settings[curr_state]));
      digitalWrite(DIRA, HIGH);   //CCW
      digitalWrite(DIRB, LOW);
      Serial.println("THIS");
    }
    else if (rotation == 2)
    {
      analogWrite(ENABLE,(255 * speed_settings[curr_state]));
      digitalWrite(DIRA, LOW); // CW
      digitalWrite(DIRB, HIGH);
      Serial.println("THERE");

    }

  if(timeFlag){
    if(curr_state == 0){
      lcd.setCursor(7, 0);
      lcd.print("0   ");
    }
    else if(curr_state == 1){
      lcd.setCursor(7, 0);
      lcd.print("1/2 ");
    }
    else if(curr_state == 2){
      lcd.setCursor(7, 0);
      lcd.print("3/4 ");
    }
    else if (curr_state == 3){
      lcd.setCursor(7, 0);
      lcd.print("Full");
    }
    
    if(rotation == 2){
      lcd.setCursor(12, 0);
      lcd.print("CW ");
    }
    else if(rotation == 1){
      lcd.setCursor(12, 0);
      lcd.print("CCW");
    }
    
    // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0):
    lcd.setCursor(0, 1);
    // print the number of seconds since reset:
    //lcd.print(millis() / 1000);
    dt = clock.getDateTime();
    lcd.print(dt.month);   lcd.print("/");
    lcd.print(dt.day);  lcd.print("/");
    lcd.print(dt.year % 100);    lcd.print(" ");
    lcd.print(dt.hour);   lcd.print(":");
    lcd.print(dt.minute); lcd.print(":");
    lcd.print(dt.second); lcd.println(" ");


    //const double speed_settings[] = {0, 0.5, 0.75, 1}; //settings for the motor's speed
    //int curr_state = 0;

    if (main_freq <= 262*1.02 && main_freq >= 262*0.98) 
    {
      if(curr_state >= 0 && curr_state < 3)
      {
        curr_state++;
        total_speed = 255 * speed_settings[curr_state];
        analogWrite(ENABLE,total_speed);
        Serial.print(255 * speed_settings[curr_state]);
        Serial.println("");
        Serial.print(rotation);
        Serial.println("");
        //Serial.print("error\n");
      }

    } 
    else if (main_freq <= a4*1.02 && main_freq >= a4*0.98) 
    {
      if(curr_state > 0 && curr_state <= 3)
      {
        curr_state--;
        analogWrite(ENABLE,255 * speed_settings[curr_state]);
        Serial.println("1:");
        Serial.print(speed_settings[curr_state]);
        Serial.println("");
        Serial.print(rotation);
        Serial.println("");
        //Serial.print("error\n");
      }
    }
    
    timeFlag = false;
  }


}

//Button INterupt
void blink() {
  //state = !state;
  if(clock1 == true)
  {
    rotation = 1;
    clock1 = false;
    //Serial.println(rotation);
  }
  else if(clock1 == false)
  {
    rotation = 2;
    clock1 = true;
    //Serial.println(rotation);
  }
  //clock1 = !clock1;
}


