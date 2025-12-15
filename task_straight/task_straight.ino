#include <NewPing.h>

// Set the pin of left motor
#define L_PWM 5
#define L_IN1 6
#define L_IN2 7
// Set the pin of right motor
#define R_PWM 10
#define R_IN1 8
#define R_IN2 9

//Set the speed of motor
#define MOTOR_BASED_SPEED 110
#define MOTOR_MAX_SPEED 255
#define MOTOR_DIFFERENT_GEAR 12

// Set the front sensor pin
#define FRONT_TRIGGER_PIN 11
#define FRONT_ECHO_PIN 12
// Set the right sensor pin
#define RIGHT_TRIGGER_PIN 3
#define RIGHT_ECHO_PIN 2

//Set the distance of sensor
#define SAFE_DISTANCE 20 // in cm
#define MAX_DISTANCE 200 // in cm

NewPing sonarFront(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance. 
NewPing sonarRight(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance. 

void setup() {
    //Setup motor pins
    pinMode(R_PWM, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(L_IN1, OUTPUT);
}

// Set motor direction
void setMotorSpeed(const int pin1, const int pin2, const int speed) {
    if (speed > 0) {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
    } else if (speed < 0) {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
    } else {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
    }
}

void stopMotor() {
    setMotorSpeed(L_IN1, L_IN2, 0);
    setMotorSpeed(L_IN1, L_IN2, 0);
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, 0);
}

// Set both motors direction
void setMotor(const int left_speed, const int right_speed) {
    int leftSpeed = constrain(left_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    int rightSpeed = constrain(right_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    //Set left motor direction
    setMotorSpeed(L_IN1, L_IN2, leftSpeed);
    //Set right motor direction
    setMotorSpeed(L_IN1, L_IN2, leftSpeed);
    //Limit speed to MOTOR_MAX_SPEED
    if (leftSpeed > MOTOR_MAX_SPEED) {
        leftSpeed = MOTOR_MAX_SPEED;
    }
    if (rightSpeed > MOTOR_MAX_SPEED) {
        rightSpeed = MOTOR_MAX_SPEED;
    }
    //Set motor speed
    analogWrite(L_PWM, leftSpeed);
    analogWrite(R_PWM, rightSpeed);
}

void loop() {
    int left_speed = 0;
    int right_speed = 0;
}