#include <Arduino.h>

const int speedPin = 5; // PWM pin for speed control
const int steeringPin = 6; // PWM pin for steering control

// Converting Throttle strength to PWM
const int speed_cmd_min = 10;
const int speed_cmd_max = 127;

const int speed_pwm_min = static_cast<int>(static_cast<float>((speed_cmd_max - speed_cmd_min)) * 2 * (3.3 / 5.0));
const int speed_pwm_max = 0;

// Converting Steering strength to PWM
const int steering_cmd_min = -128;
const int steering_cmd_zero = 40;
const int steering_cmd_max = 100;

const int steering_pwm_min = 0;
const int steering_pwm_zero = static_cast<int>(static_cast<float>((steering_cmd_zero - steering_cmd_min)) * (3.3 / 5.0));
const int steering_pwm_max = static_cast<int>(static_cast<float>((steering_cmd_max - steering_cmd_min)) * (3.3 / 5.0));

void setup() {
  Serial.begin(115200);
  pinMode(speedPin, OUTPUT);
  pinMode(steeringPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (Serial.available() >= 8) { // Check if there's enough data available to read
    // Read incoming data
    uint8_t buffer[8];
    Serial.readBytes(buffer, 8);

    // Parse received data
    char speed, steering;
    memcpy(&speed, &buffer[0], sizeof(char));
    memcpy(&steering, &buffer[sizeof(char)], sizeof(char));

    // Map speed and steering values to the appropriate analog output range
    int mappedSpeed = map(static_cast<int>(speed), 0, 127, speed_pwm_min, speed_pwm_max);

    int mappedSteering = steering >= 0 ? map(static_cast<int>(steering), 0, 127, steering_pwm_zero, steering_pwm_max) : 
                                                        map(static_cast<int>(steering), -128, 0, steering_pwm_min, steering_pwm_zero); // Map steering to 0-3.3V range

    // Write mapped values to PWM pins
    analogWrite(speedPin, mappedSpeed);
    analogWrite(steeringPin, mappedSteering);

    // Control LED_BUILTIN based on speed value
    int brightness = map(static_cast<int>(speed), 0, 127, 0, 255); // Map speed to LED brightness
    analogWrite(LED_BUILTIN, brightness);

    // Relay the data back through serial
    Serial.print('!'); // Start character
    Serial.print(static_cast<char>(speed)); // Speed value as char
    Serial.print(static_cast<char>(steering)); // Steering value as char
    Serial.println(); // End of message
  }
}
