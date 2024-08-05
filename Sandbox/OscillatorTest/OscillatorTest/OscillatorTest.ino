#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define PWM_PIN 15  // The pin where the PWM signal is connected
#define SERVO_FREQ 50

unsigned long startTime;
unsigned long endTime;
unsigned long period;

void setup() {
  Serial.begin(115200);
  while (Serial.available() == 0) {}
  Serial.println("Begin PWM Oscillator Test!");
  pwm.begin();

  // 23 MHz = 55.2 Hz
  // 24 MHz = 
  // 25 Mhz = 50.68 Hz
  // 25.225 MHz = 50.28 Hz
  // 25.25 MHz = 50.27 Hz
  // 25.28 MHz = 50.28 Hz
  // 25.30 MHz = 49.87 Hz
  // 25.35 MHz = 
  // 25.5 MHz = 49.47 Hz
  // 26 Mhz = 
  // 27 MHz = 46.84 Hz
  
  pwm.setOscillatorFrequency(25250000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  
  pinMode(PWM_PIN, INPUT);
}

void loop() {
  Serial.println("Setting PWM for pin: 0");
  pwm.setPWM(0, 0, 600);
  // Wait for the signal to go HIGH
  while (digitalRead(PWM_PIN) == LOW) {}
  startTime = micros();  // Record the start time
  
  // Wait for the signal to go LOW
  while (digitalRead(PWM_PIN) == HIGH) {}
  
  // Wait for the signal to go HIGH again
  while (digitalRead(PWM_PIN) == LOW) {}
  endTime = micros();  // Record the end time
  
  // Calculate the period of the PWM signal
  period = endTime - startTime;
  
  // Calculate the frequency (Hz)
  float frequency = 1000000.0 / period;
  
  // Print the frequency
  Serial.print("PWM Frequency: ");
  Serial.print(frequency);
  Serial.println(" Hz");
  
  delay(1000);  // Wait a second before measuring again
}
