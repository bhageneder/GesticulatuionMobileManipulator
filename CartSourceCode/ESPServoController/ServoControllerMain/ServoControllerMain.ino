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
#define SERVOCW  410 
#define SERVOSTOP  600 
#define SERVOCOUNTERCW 205  

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  Serial.println("Servo test!");

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

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~50 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void testServoPulse() {
  Serial.println("Foward");
  setServoPulse(0, 0.002);
  setServoPulse(1, 0.001);
  delay(2000);

  Serial.println("Stop");
  pwm.setPWM(0, 0, SERVOSTOP);
  pwm.setPWM(1, 0, SERVOSTOP);
  delay(2000);

  Serial.println("Reverse");
  setServoPulse(0, 0.001);
  setServoPulse(1, 0.002);
  delay(2000);

  Serial.println("Stop");
  pwm.setPWM(0, 0, SERVOSTOP);
  pwm.setPWM(1, 0, SERVOSTOP);
  delay(2000);
}

void testServoLimits() {
  Serial.println("Foward");
  Serial.println(SERVOCW);
  Serial.println(SERVOSTOP);
  pwm.setPWM(0, 1, SERVOCW);
  pwm.setPWM(1, 0, SERVOCOUNTERCW);
  delay(2000);

  Serial.println("Stop");
  pwm.setPWM(0, 0, SERVOSTOP);
  pwm.setPWM(1, 0, SERVOSTOP);
  delay(2000);

  Serial.println("Reverse");
  pwm.setPWM(0, 0, SERVOCOUNTERCW);
  pwm.setPWM(1, 1, SERVOCW);
  delay(2000);

  Serial.println("Stop");
  pwm.setPWM(0, 0, SERVOSTOP);
  pwm.setPWM(1, 0, SERVOSTOP);
  delay(2000);
}


void loop() {
  //testServoPulse();

  testServoLimits();
}
