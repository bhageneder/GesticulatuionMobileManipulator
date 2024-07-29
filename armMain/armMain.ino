// ----------------------------------------------------------
// Load Libraries
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// ----------------------------------------------------------

// ----------------------------------------------------------
// Create global class variables
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // uses the default address 0x40
// ----------------------------------------------------------

// ----------------------------------------------------------
// Creating global fields
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVOMIN 250 // Minimum Servo length
#define SERVOMAX 500 // Maximum Servo PWM length
// ----------------------------------------------------------

// ----------------------------------------------------------
// Method to transmit information back to external device / other esp to be read
void sendMessage(String message) {
  // Temporary serial print message
  Serial.println(message);
  // TODO: Create algorithm to send data
}
// ----------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  sendMessage("Setting up servos.");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMfreq(SERVO_FREQ);
  sendMessage("Servo setup complete!");
}

int mapAngleToPWM(degree) {
  return pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
}

void spinRight() {
  sendMessage("Spinning Right");
  pwm.setPWM(0, 0, SERVOCW);
}

void turnLeft() {
  sendMessage("Spinning Left");
  pwm.setPWM(0, 0, SERVOCOUNTERCW);
}

void moveStop() {
  sendMessage("Stoping Movement");
  pwm.setPWM(0, 0, SERVOSTOP);
}

void loop() {
  // put your main code here, to run repeatedly:

}
