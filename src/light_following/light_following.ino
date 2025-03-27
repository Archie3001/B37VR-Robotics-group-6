//light following code for B37VR
//group 6{
//Archie Speake,
//??,
//?? }
//created/modified with reference from the provided file "sketch_light_follow_new_robot(1).txt"
//copyright from said file
/*====================================================================================*|
// WheelDrive - move a pair of DC motors at varying rate and direction
//
// Copyright (c) 2016, Garth Zeglin.  All rights reserved. Licensed under the
// terms of the BSD 3-clause license as included in LICENSE.
//
// This program assumes that:
//
//  1. A DRV8833 dual DC motor driver module is connected to pins 5, 6, 9, and 10.
//  2. A pair of motors is attached to the driver.
//  3. The serial console on the Arduino IDE is set to 9600 baud communications speed.
|*====================================================================================*/
 
////defining I/O pins:
//Input:
const int sensorPinL = A1;
const int sensorPinR = A2;
//Output:
#define MOTOR_A_PIN1 5
#define MOTOR_A_PIN2 6
#define MOTOR_B_PIN1 9
#define MOTOR_B_PIN2 10
////end of I/O pin definitions

////defining global variables
//constants:
const int sensorDiffThresh = 50;
const int slowSpeed = 100; //must be within range(0,255)
const int fastSpeed = 250; //must be within range(0,255)
//variables:
int sensorReadL = 0;
int sensorReadR = 0;
int sensorDiff  = 0;
int servoSpeedL = 0;
int servoSpeedR = 0;
////end of global variable declarations

////===========================SETUP================================
//runs once on startup; Or after pressing reset; Or after powering on
void setup() {
  
  // Initialize the stepper driver control pins to output drive mode.
  pinMode(MOTOR_A_PIN1, OUTPUT);
  pinMode(MOTOR_A_PIN2, OUTPUT);
  pinMode(MOTOR_B_PIN1, OUTPUT);
  pinMode(MOTOR_B_PIN2, OUTPUT);

  // Start with drivers off, motors coasting.
  digitalWrite(MOTOR_A_PIN1, LOW);
  digitalWrite(MOTOR_A_PIN2, LOW);
  digitalWrite(MOTOR_B_PIN1, LOW);
  digitalWrite(MOTOR_B_PIN2, LOW);

  // Initialize the serial UART at 9600 bits per second.
  Serial.begin(9600);

  //end of setup
}


// ================================================================================
/// Set the current on a motor channel using PWM and directional logic.
/// Changing the current will affect the motor speed, but please note this is
/// not a calibrated speed control.  This function will configure the pin output
/// state and return.
///
/// \param pwm    PWM duty cycle ranging from -255 full reverse to 255 full forward
/// \param IN1_PIN  pin number xIN1 for the given channel
/// \param IN2_PIN  pin number xIN2 for the given channel

void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}
// ================================================================================
/// Set the current on both motors.
///
/// \param pwm_A  motor A PWM, -255 to 255
/// \param pwm_B  motor B PWM, -255 to 255

void set_motor_currents(int pwm_A, int pwm_B)
{
  set_motor_pwm(pwm_A, MOTOR_A_PIN1, MOTOR_A_PIN2);
  set_motor_pwm(pwm_B, MOTOR_B_PIN1, MOTOR_B_PIN2);

  // Print a status message to the console.
  Serial.print("Set motor A PWM = ");
  Serial.print(pwm_A);
  Serial.print(" motor B PWM = ");
  Serial.println(pwm_B);
}
// ================================================================================
/// Simple primitive for the motion sequence to set a speed and wait for an interval.
///
/// \param pwm_A  motor A PWM, -255 to 255
/// \param pwm_B  motor B PWM, -255 to 255
/// \param duration delay in milliseconds
void spin_and_wait(int pwm_A, int pwm_B, int duration)
{
  set_motor_currents(pwm_A, pwm_B);
  delay(duration);
}



////===========================LOOP=================================
//loops after setup() is finished running
void loop() {

  ////taking sensor readings:
  //Left sensor:
  sensorReadL = 1023 - analogRead(sensorPinL);
  delay(1); // delay to let adc settle
  //Right sensor:
  sensorReadR = 1023 - analogRead(sensorPinR);
  delay(1); // delay to let adc settle
  ////sensor readings have been taken.

  sensorDiff = abs(sensorReadL - sensorReadR);
  
  ////deciding servo/motor speeds
  if (sensorDiff < sensorDiffThresh){
    //go straight forwards
    servoSpeedL = fastSpeed;
    servoSpeedR = fastSpeed;
    Serial.print("going forwards");
  }
  else if (sensorReadL > sensorReadR){
    //turn left
    servoSpeedL = slowSpeed;
    servoSpeedR = fastSpeed;
    Serial.print("going left");
  }
  else{// (sensorReadL < sensorReadR)
    //turn right
    servoSpeedL = fastSpeed;
    servoSpeedR = slowSpeed;
    Serial.print("going right");
  }
  //end if
  ////servo/motor speeds are decided

  spin_and_wait(servoSpeedL,servoSpeedR,500);// sets motor/servo speeds for the next 500 ms
  
  //end of loop()
}
