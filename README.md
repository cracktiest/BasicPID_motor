# BasicPID_motor

Basic implementation of a PID (Proportional-Integral-Derivative) controller for controlling motor position based on a setpoint using an incremental rotary encoder.

### Description
This project demonstrates a PID controller implementation in Arduino to control motor positioning. The motor's position is adjusted based on the difference between the target position and the encoder reading. The code is tested on Arduino Mega 2560 and has been verified to work effectively, providing precise motor position control.

### Requirements
- **Hardware**: Arduino Mega 2560 (or compatible board), motor driver, incremental rotary encoder
- **Libraries**: No additional libraries are required.
- **Components**:
  - Motor connected to pins `PWM`, `IN1`, and `IN2`
  - Encoder connected to pins 2 and 3 (with interrupts)

### Pin Configuration
- `PWM` (pin 8): PWM pin for motor control
- `IN1` (pin 10): Motor input pin 1
- `IN2` (pin 9): Motor input pin 2
- `ENCODER_A` (pin 2): Encoder channel A
- `ENCODER_B` (pin 3): Encoder channel B

### PID Controller Explanation
The PID controller adjusts the motor speed and direction based on the following three components:

1. **Proportional (P)**: The proportional component calculates the error, \( e \), which is the difference between the target position and the current position:
   \[
   e = \text{target} - \text{position}
   \]
   The control signal is directly proportional to the error, scaled by a proportional gain \( K_p \). This component provides a response to minimize the error.

2. **Integral (I)**: The integral component accumulates the error over time, summing up past errors to address any steady-state offset. It is calculated as:
   \[
   \text{eintegral} = \text{eintegral} + e \times \Delta T
   \]
   where \( \Delta T \) is the time interval since the last update. The integral term, multiplied by \( K_i \), helps the motor reach the exact target by reducing any persistent, small errors.

3. **Derivative (D)**: The derivative component predicts the system's behavior by calculating the rate of change of the error:
   \[
   \frac{de}{dt} = \frac{e - e_{\text{prev}}}{\Delta T}
   \]
   where \( e_{\text{prev}} \) is the previous error. This component, scaled by \( K_d \), helps reduce overshooting by smoothing out rapid changes in the error.

The overall control signal \( u \) is given by:
\[
u = K_p \times e + K_i \times \text{eintegral} + K_d \times \frac{de}{dt}
\]

### Configuration and Compilation Details
- **RAM Usage**: 15.5% (317 bytes of 2048 bytes)
- **Flash Usage**: 15.7% (5068 bytes of 32256 bytes)
- **Platform**: Atmel AVR (4.0.0)
- **Build Time**: 7.56 seconds
- **Board Configuration**: [PlatformIO - Arduino Uno Configuration](https://docs.platformio.org/page/boards/atmelavr/uno.html)
- **Hardware**: ATMEGA328P 16MHz, 2KB RAM, 31.50KB Flash

### Code Structure
- **`setMotor(dir, pwmVal, pwm, in1, in2)`**: Sets the motor direction and speed.
- **`loop()`**: Main loop runs the PID control to drive the motor towards the target position.
- **`ai0()` and `ai1()`**: Interrupt service routines for the encoder, triggered on pin changes to update position.
- **`setup()`**: Initializes the Serial, pins, and encoder interrupts.

```cpp
#include <Arduino.h>
#define PWM 8 // PWM pin for motor
#define IN2 9 
#define IN1 10

volatile unsigned int temp, counter = 0;  // Counter for encoder
volatile int posi = 0;  // Encoder position
long prevT, currT = 0;
float eprev, eintegral, pwr, deltaT = 0;
int e;
int target = 2023; // Target position

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
  // PID control logic
}

void ai0() {
  // Increment or decrement position based on encoder direction
}

void ai1() {
  // Increment or decrement position based on encoder direction
}

void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, ai0, RISING);
  attachInterrupt(1, ai1, RISING);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  Serial.println("target pos");
}
