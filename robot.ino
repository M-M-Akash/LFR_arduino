#include <ResponsiveAnalogRead.h>

#define Kp 80
#define Kd 0
const int pins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int sensors[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int white[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int black[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int locations[8];
int error = 0;
int lastError = 0;
#define emitter_pin 2

#define MOT1 28
#define MOT2 26
#define MOT3 30
#define MOT4 32

int baseSpeed = 75;
int rightMotorSpeed = 0;
int leftMotorSpeed = 0;
void setup() {
  Serial.begin(9600);
  pinMode (MOT1, OUTPUT);
  pinMode (MOT2, OUTPUT);
  pinMode (MOT3, OUTPUT);
  pinMode (MOT4, OUTPUT);
  
  sens_location();
  
  for (int i = 0; i < 40; i++) {
    
    ( i  < 10 || i >= 20 ) ? MotorSpeed(150, -150) : MotorSpeed(-150, 150);
    
    calibration();
    delay(20);
  }
  
   MotorSpeed(0,0);
  delay(1000);

}

void loop() {
  read_sensors();
  pid_calc();
  //MotorSpeed (leftMotorSpeed,rightMotorSpeed);
}
void read_sensors() {
  int numerator = 0;
  int denominator = 0;

  for (int i = 0; i < 8; i++) {
    int a = 0, b = 0;
    ResponsiveAnalogRead analog(pins[i], true);
    analog.update();
    /*digitalWrite(emitter_pin, HIGH); //glow the emitter for 500 microseconds
      delayMicroseconds(500);
      a = analog.getValue();

      digitalWrite(emitter_pin, LOW); //turn off the emitter for 500 microseconds
      delayMicroseconds(500);
      b = analog.getValue();

      sensors[i] = a - b;
    */
    sensors[i] = analog.getValue();
    sensors[i] = (sensors[i] < black[i]) ? black[i] : sensors[i];
    sensors[i] = (sensors[i] > white[i]) ? white[i] : sensors[i];
    sensors[i] = ((sensors[i] - black[i]) / (white[i] - black[i])) * 100;
    numerator += sensors[i] * locations[i];
    denominator += sensors[i];

  }

  error = (numerator == 0) ? 0 : numerator / (10 * denominator);
  //Serial.println(error);
}
void calibration() {
  for (int i = 0; i < 8; i++) {
    int a = 0, b = 0;
    ResponsiveAnalogRead analog(pins[i], true);
    analog.update();
    /*digitalWrite(emitter_pin, HIGH); //glow the emitter for 500 microseconds
      delayMicroseconds(500);
      a = analog.getValue();

      digitalWrite(emitter_pin, LOW); //turn off the emitter for 500 microseconds
      delayMicroseconds(500);
      b = analog.getValue();

      sensors[i] = a - b;
    */
    sensors[i] = analog.getValue();
    white[i] = (sensors[i] > white[i]) ? sensors[i] : white[i];
    black[i] = (sensors[i] < black[i]) ? sensors[i] : black[i];
     Serial.print(i);
      Serial.print("\t");
      Serial.print(white[i]);
      Serial.print("\t");
      Serial.println(black[i]);
  }
}
void sens_location() {
  for (int i = 0; i < 8; i++) {
    locations[i] = (-40) + (i * 80 / 7); // sensor array is 8cm/80mm there are 7 blank spaces between 8 sensors  the most left sensor location is -40

  }
}


void pid_calc() {
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  rightMotorSpeed = baseSpeed-motorSpeed;
  leftMotorSpeed = baseSpeed + motorSpeed;
  /*Serial.print(error);
  Serial.print("\t");
  Serial.print(rightMotorSpeed);
  Serial.print("\t");
  Serial.println(leftMotorSpeed);*/
}

void MotorSpeed (int leftmotor, int rightmotor) { 
  int en1 = abs (leftmotor);
  int en2 = abs (rightmotor);
  en1 = (leftmotor > 0) ? 200 - en1 : en1;
  en2 = (rightmotor >= 0) ? en2 : 200 - en2;
  analogWrite (MOT2, en1);
  analogWrite (MOT3, en2);
  analogWrite (MOT1, (leftmotor > 0) ? 200 : 0);
  analogWrite (MOT4, (rightmotor >= 0) ? 0 : 200);
}
