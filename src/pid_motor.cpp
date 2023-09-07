#include <Arduino.h>
#define PWM 8 //pin PWM for motor
#define IN2 9 
#define IN1 10

volatile unsigned int temp, counter = 0;  //This variable will increase or decrease depending on the rotation of encoder
volatile int posi = 0;  // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT,currT = 0;
float eprev,eintegral,pwr,deltaT = 0;
int e;
int target = 2023; //setpoint for the system to read encoder's step and run by motor



void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


void loop() {
  for (int ulang = 1; ulang <= 10; ulang++) {
    posi = 0;
    currT = 0;
    deltaT = 0;
    e = 0;
    pwr = 0;
    for (int osi = 20; osi >= 5;) {
      float kp = 1;
      float kd = 0.05;
      float ki = 0.00002;

      // time difference
      currT = micros();
      deltaT = ((float)(currT - prevT)) / (1.0e6);
      prevT = currT;

      // Read the position
      int pos = 0;
      noInterrupts();  // disable interrupts temporarily while reading
      pos = posi;
      interrupts();  // turn interrupts back on

      // error
      e = pos - target;

      // derivative
      float dedt = (e - eprev) / (deltaT);

      // integral
      eintegral = eintegral + e * deltaT;

      // control signal
      float u = kp * e + kd * dedt + ki * eintegral;

      // motor direction
      int dir = 1;
      if (u < 0) {
        dir = -1;
      }

      // motor power
      pwr = fabs(u);
      if (pwr > 255) {
        pwr = 255;
      } else if (pwr <= 54 && pwr >= 2) {
        pwr = 55;
      }

      // signal the motor


      // store previous error
      eprev = e;

      //break the loop osi
      if ((target - pos) >= -5 && (target - pos) <= 5) {
        osi=osi-5;
        dir = 2;
      } else {
        osi++;
      }

      setMotor(dir, pwr, PWM, IN1, IN2);
      Serial.print(target);
      Serial.print(" ");
      Serial.print(pos);
      Serial.print(" ");
      Serial.print(pwr);
      Serial.print(" ");
      Serial.print(osi);
      Serial.println();
    }
  }
  int dir = 2;
  setMotor(dir, pwr, PWM, IN1, IN2);
  delay(10000);
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(3) == LOW) {
    posi--;
  } else {
    posi++;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(2) == LOW) {
    posi++;
  } else {
    posi--;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(2, INPUT_PULLUP);  // internal pullup input pin 2

  pinMode(3, INPUT_PULLUP);  // internalเป็น pullup input pin 3
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);

  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.println("target pos");
}
