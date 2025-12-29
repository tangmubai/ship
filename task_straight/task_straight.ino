#include <NewPing.h>

// Set the pin of right motor
#define L_PWM 11
#define L_IN1 7
#define L_IN2 6
// Set the pin of left motor
#define R_PWM 3
#define R_IN3 4
#define R_IN4 5

//Set the speed of motor
#define MOTOR_BASED_SPEED 250
#define MOTOR_MAX_SPEED 255
#define BASED_DIFFERENT_GEAR 50

void setup() {
    //Setup motor pins
    pinMode(R_PWM, OUTPUT);
    pinMode(R_IN3, OUTPUT);
    pinMode(R_IN4, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(L_IN1, OUTPUT);
    //Stop motors at the beginning
    stopMotor();
    //Setup Serial port
    Serial.begin(9600);
    Serial.println("Task Straight Start");

}

// Set motor direction
int setMotorSpeed(const int pin1, const int pin2, const int PMW, const int speed) {
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
    int setSpeed = abs(speed);
    if (setSpeed > MOTOR_MAX_SPEED) {
        setSpeed = MOTOR_MAX_SPEED;
    }
    analogWrite(PMW, setSpeed);
    return setSpeed;
}

// Stop both motors
void stopMotor() {
    setMotorSpeed(L_IN1, L_IN2, L_PWM, 0);
    setMotorSpeed(R_IN3, R_IN4, R_PWM, 0);
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, 0);
}

// Set both motors direction
void setMotor(const int left_speed, const int right_speed) {
    int leftSpeed = constrain(left_speed, 0, MOTOR_MAX_SPEED);
    int rightSpeed = constrain(right_speed, 0, MOTOR_MAX_SPEED);
    //Set left motor direction
    leftSpeed = setMotorSpeed(L_IN1, L_IN2, L_PWM, leftSpeed);
    //Set right motor direction
    rightSpeed = setMotorSpeed(R_IN3, R_IN4, R_PWM, rightSpeed);
    //Limit speed to MOTOR_MAX_SPEED
    Serial.print("Left Motor Speed: ");
    Serial.print(leftSpeed);
    Serial.print("\tRight Motor Speed: ");
    Serial.println(rightSpeed);
}

void loop() {
    setMotor(MOTOR_BASED_SPEED - BASED_DIFFERENT_GEAR, MOTOR_BASED_SPEED);
}