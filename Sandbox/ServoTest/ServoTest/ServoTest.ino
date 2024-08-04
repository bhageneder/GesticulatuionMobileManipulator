/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  125 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  475 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}


void loop() {
  // while (Serial.available() == 0) {}
  // int pulselen = Serial.parseInt();

  // vertical
  // servo min: 130
  // servo max: 475

  // horizontal:
  // servo min: 70
  // servo max: 500

  // heavy duty:
  // servo min: 250
  // servo max: 500
  
  // Drive each servo one at a time using setPWM()
  Serial.println("Enter servo to control: ");
  while (Serial.available() == 0) {
    // Wait for input
  }
  int servonum = Serial.readStringUntil('\n').toInt();
  Serial.flush();
  delay(500);
  
  Serial.println("Enter pulselength to use: ");
  while (Serial.available() == 0) {
    // Wait for input
  }
  int pulselen = Serial.readStringUntil('\n').toInt();
  Serial.flush();
  delay(500);
  Serial.print("The pulselen is: ");
  Serial.print(pulselen);
  Serial.println();
  pwm.setPWM(servonum, 0, pulselen);

  // Serial.println("Enter degree to turn to: ");
  // while (Serial.available() == 0) {
  //   // Wait for input
  // }
  // int degree = Serial.readStringUntil('\n').toInt();
  // Serial.flush();
  // delay(500);
  // int pulselen = map(degree, 0, 270, SERVOMIN, SERVOMAX);

  // Serial.print("Degree entered is: ");
  // Serial.print(degree);
  // Serial.println();
  // Serial.print("Corresponding pulselength is: ");
  // Serial.print(pulselen);
  // Serial.println();
  // pwm.setPWM(servonum, 0, pulselen);
}
