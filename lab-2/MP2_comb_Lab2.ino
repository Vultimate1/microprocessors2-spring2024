//Link to Youtube video where I found the template for this code https://www.youtube.com/watch?v=a37xWuNJsQI for the gyro section
//Link in YT vid to MPU6050 Library:  http://bit.ly/39hycAR
//Link in YT vid to MPU6050 code:  http://bit.ly/2MQD9cp
//Li

#include "Wire.h"       
#include "I2Cdev.h"     
#include "MPU6050.h"    

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

struct MyData {
  byte X;
  byte Y;
  byte Z;
};

MyData data;


// LED status
int led_status = HIGH;
int incomingByte = 0;

// Arduino pin numbers
const int SW_pin = 2; // digital pin connected to switch output
const int X_pin = 0; // analog pin connected to X output
const int Y_pin = 1; // analog pin connected to Y output
const int buzzer = 7; //Digital pin connected to Buzzer output

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);
  pinMode(buzzer, OUTPUT);
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

}

// the loop routine runs over and over again forever:
void loop() {
  int Xaxis = analogRead(X_pin);
  int Yaxis = analogRead(Y_pin);

  if(Yaxis == 0)
  {
    Serial.print("w\n");
    //UP
    delay(500);
  }
  else if(Yaxis == 1023)
  {
    Serial.print("s\n");
    //Down
    delay(500);
  }
  
  if(Xaxis == 1023)
  {
    Serial.print("d\n");
    //Right
    delay(500);
  }
  else if(Xaxis == 0)
  {
    Serial.print("a\n");
    //Left
    delay(500);
  }


  //Gryo Code
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  data.X = map(ax, -17000, 17000, 0, 255 ); // X axis data
  data.Y = map(ay, -17000, 17000, 0, 255); 
  data.Z = map(az, -17000, 17000, 0, 255);  // Y axis data
  delay(1000);

  if(data.X < 115 && (data.Y <= 140 && data.Y >= 120))
  {
    Serial.print("d\n");
    //Go Right
  }
  else if(data.X > 135 && (data.Y <= 140 && data.Y >= 120))
  {
    Serial.print("a\n");
    //Go left
  }

  if(data.Y > 135 && (data.X <= 140 && data.X >= 120))
  {
    Serial.print("s\n");
    //Go down
  }
  else if(data.Y < 120 && (data.X <= 150 && data.X >= 110))
  {
    Serial.print("w\n");
    //Go forward
  }

  

  if(Serial.available() > 0)
  {
    incomingByte = Serial.read();
    Serial.println(incomingByte);

    if(incomingByte == 'E')
    {
      //BUzz the buzzer
      digitalWrite(buzzer, HIGH);
      delay(500);
      digitalWrite(buzzer, LOW);
    }
  }

}

//first run connect arduino and make sure both sides are working together
