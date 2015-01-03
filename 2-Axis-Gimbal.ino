#include <ADXL345.h>
#include <bma180.h>
#include <HMC58X3.h>
#include <ITG3200.h>
#include <MS561101BA.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <EEPROM.h>
#include "DebugUtils.h"
#include "FreeIMU.h"
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <Servo.h>

Servo servo_X;
Servo servo_Y;
int raw_values[11];
char str[512];
float val[9];
float acceleration[3];
float rotation[3];
float degree[3];
float ypr[3];
const float pi_2=1570796;
int elojel;
int hatso_elteres;
int rangas=0;
int base_speed = 0;
int mintavetel = 0;
int fok = 90;
int degreecounter = 1;
int degreecounterX;
int degreecounterY;
int counterX;
int counterY;
float new_timeCounter;
float last_timeCounter = 0;
float currInterval = 0.f;
float maxInterval = 1000.f / 22.f;
unsigned long time;
int velX;
int velY;
int nextX;
int nextY;
FreeIMU my3IMU = FreeIMU();




void setup() 
{  
  servo_X.attach(9);
  servo_Y.attach(10);
  servo_X.write(90);
  servo_Y.write(90);
  Serial.begin(115200);
  Wire.begin();
  delay(1500);
  my3IMU.init(true);
  delay(1500);
}


float timerElapsed(void)
{
  //Serial.print("Time: ");
  time = micros();
  //prints time since program started
  //Serial.println(time);
  return time;
}


void loop() {
  my3IMU.getRawValues(raw_values);
  my3IMU.getYawPitchRoll(ypr);
  acceleration[0]=(map(raw_values[0],-32768,32768,-1962,1962))/100.00;
  acceleration[1]=(map(raw_values[1],-32768,32768,-1962,1962))/100.00;
  acceleration[2]=(map(raw_values[2],-32768,32768,-1962,1962))/100.00;
  rotation[0]=map(raw_values[3],-32768,32768,-250,250);
  rotation[1]=map(raw_values[4],-32768,32768,-250,250);
  rotation[2]=map(raw_values[5],-32768,32768,-250,250);
  mintavetel = mintavetel + 1;
  new_timeCounter = timerElapsed();
  currInterval+=(new_timeCounter-last_timeCounter) / 1000;
  
  if ( currInterval >= maxInterval )
  {
    currInterval = 0.f;
    //if (mintavetel == 1)
  //{
    Serial.println(new_timeCounter);
    Serial.println("currentinterval: ");
    Serial.println(currInterval);
        if(acceleration[1]==0)
        {
          degree[1]=0;
        }
      else
        {
          if(acceleration[2]==0)
            {
              if(acceleration[1]<0)
                {
                  degree[1]=-90;
                }
              else
                {
                  degree[1]=90;
                }
            }
          else
            {         
              degree[1]=atan(acceleration[2]/acceleration[1]);
              if (degree[1]<0)
                {
                  elojel=1;
                }
              else
                {
                  elojel=0;
                }
              degree[1]=90-abs(map(degree[1]*1000000,-pi_2,pi_2,-90,90));
              if (elojel==1)
                {
                  degree[1]=degree[1]*(-1);
                }
            }
        }
        
      if(acceleration[0]==0)
        {
          degree[0]=0;
        }
      else
        {
          if(acceleration[2]==0)
            {
              if(acceleration[0]<0)
                {
                  degree[0]=-90;
                }
              else
                {
                  degree[0]=90;
                }
            }
          else
            {         
              degree[0]=atan(acceleration[2]/acceleration[0]);
              if (degree[0]<0)
                {
                  elojel=1;
                }
              else
                {
                  elojel=0;
                }
              degree[0]=90-abs(map(degree[0]*1000000,-pi_2,pi_2,-90,90));
              if (elojel==1)
                {
                  degree[0]=degree[0]*(-1);
                }
            }
        }
       if((degree[0]-4) > 15)
         {
           hatso_elteres=15;
         }
       if((degree[0]-4)<(-15))
         {
            hatso_elteres=-15;
         }
       else
          {
            hatso_elteres=degree[0]-4;
          }
      Serial.println("Acceleration:");
      Serial.print("X axis: ");
      Serial.print(acceleration[0]);
      Serial.println(" m/(s^2)");
      Serial.print("Y axis: ");
      Serial.print(acceleration[1]);
      Serial.println(" m/(s^2)");
      Serial.print("Z axis: ");
      Serial.print(acceleration[2]);
      Serial.println(" m/(s^2)");
      
      Serial.println("Rotational velocity:"); 
      Serial.print("X axis: ");
      Serial.print(rotation[0]);
      Serial.println(" degree/s");
      Serial.print("Y axis: ");
      Serial.print(rotation[1]);
      Serial.println(" degree/s");
      Serial.print("Z axis: ");
      Serial.print(rotation[2]);
      Serial.println(" degree/s");
      
      Serial.print("X degree: ");
      Serial.println(degree[0]);
      Serial.print("Y degree: ");
      Serial.println(degree[1]);
       
      Serial.print("Sampling time: ");
      Serial.println((new_timeCounter-last_timeCounter) / 1000);
      
      Serial.print("Hatso elteres: ");
      Serial.println(hatso_elteres);
      
      
      Serial.println('\n');
  last_timeCounter = new_timeCounter;
  }
    
      counterX=counterX*0.9+0.1*(degree[0]+90);
      counterY=counterY*0.9+(90-degree[1])*0.1;
    
    servo_X.write(counterX);
    servo_Y.write(counterY);
}
