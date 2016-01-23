#include <QTRSensors.h>
#define NUM_SENSORS   8 
#define TIMEOUT       2500
#define EMITTER_PIN   2

int PWMA=5; //PWMA For Motor A
int PWMB=6; //PWMA For Motor B
int lasterror=0;
int lms=150;
int rms=150;


QTRSensorsRC qtrrc((unsigned char[]) {2,3, 4, 7, 8, 9, 10, 11},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  pinMode(A0,OUTPUT); //MOTOR AIN1
  pinMode(A1,OUTPUT); //MOTOR AIN2
  pinMode(A2,OUTPUT); //MOTOR BIN1
  pinMode(A3,OUTPUT); //MOTOR BIN2
  pinMode(A4,OUTPUT); //STBY
  pinMode(PWMA,OUTPUT); //PWMA
  pinMode(PWMB,OUTPUT); //PWMB
  digitalWrite(A4,HIGH); //STBY HIGH - Enable MOTOR Driver
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
 /* for (int i = 0; i < NUM_SENSORS; i++)
  {
  //  Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
  //  Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  */
  delay(1000);
}



void loop()
{
unsigned int sensors[8];
int position = qtrrc.readLine(sensors);

//Serial.println(position);
int error = (position - 3200);
Serial.print(error);
int lbs=100;
int rbs= 100;
float kp=0.1;
float kd=5;
int motorspeed  = (kp*error) + (kd*(error-lasterror));

lasterror =error;

int m1speed= lbs + motorspeed;
int m2speed= rbs + motorspeed;

if(m1speed > lms){ m1speed = lms;}
if(m2speed > rms){ m2speed = rms;}
if(m1speed < 0){ m1speed =0;}
if(m2speed < 0){ m2speed =0;}



digitalWrite(A0,HIGH);
digitalWrite(A1,LOW);
analogWrite(PWMA,m1speed);
digitalWrite(A2,HIGH);
digitalWrite(A3,LOW); 
analogWrite(PWMB,m2speed);
delay(100);
}
 
 
 /* TEST CASE for both MOTOR 
 
 digitalWrite(A0,HIGH);
 digitalWrite(A1,LOW);
 analogWrite(PWMA,255);
 digitalWrite(A2,HIGH);
 digitalWrite(A3,LOW); 
 analogWrite(PWMB,255); */
 
 /* Without PID
 
if (error < 2500) // the line is on the left
{
lms = 50; // turn left
 digitalWrite(A0,HIGH);
 digitalWrite(A1,LOW);
 analogWrite(PWMA,lms);
 digitalWrite(A2,HIGH);
 digitalWrite(A3,LOW); 
 analogWrite(PWMB,rms);
}
if (error > 2500) // the line is on the right
{
  rms= 50;
 digitalWrite(A0,HIGH);
 digitalWrite(A1,LOW);
 analogWrite(PWMA,lms);
 digitalWrite(A2,HIGH);
 digitalWrite(A3,LOW); 
 analogWrite(PWMB,rms);

}
 
 */
 
 /*plain code
 if (error < 0) // the line is on the left
{
 m1speed=0; // turn left
}
if (error > 0) // the line is on the right
{
  m2speed=0;
}
*/
