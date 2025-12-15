#include <NewPing.h>

// Set the pin of left motor
#define L_PWM 9
#define L_IN1 13
#define L_IN2 12
// Set the pin of right motor
#define R_PWM 8
#define R_IN1 11
#define R_IN2 10

//Set the speed of motor
#define MOTOR_BASED_SPEED 110
#define MOTOR_MAX_SPEED 255
#define BASED_DIFFERENT_GEAR 12

// Set the front sensor pin
#define FRONT_TRIGGER_PIN 11
#define FRONT_ECHO_PIN 12
// Set the right sensor pin
#define RIGHT_TRIGGER_PIN 3
#define RIGHT_ECHO_PIN 2

//Set the distance of sensor
#define SAFE_DISTANCE 20 // in cm
#define MAX_DISTANCE 200 // in cm
#define TOLLRENT_DISTANCE 10 // in cm

// Set the sonar reading parameters
#define WAIT_TIME 100 // in ms
#define STOP_TIME 50 // in ms
#define READ_ROUNDS 5

void setup() {
    //Setup motor pins
    pinMode(R_PWM, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(L_IN1, OUTPUT);
    //Stop motors at the beginning
    stopMotor();
    //Setup Serial port
    Serial.begin(9600);
    Serial.println("Task Straight Start");
}

unsigned int getSonarDistance(NewPing &sonar) {
    unsigned int sumDistance = 0;
    int vailadReadings = 0;
    for (int i = 0; i < READ_ROUNDS; i++) {
        unsigned int distance = sonar.ping_cm();
        if (distance >= 0) {
            sumDistance += distance;
            vailadReadings++;
        }
        delay(WAIT_TIME); //Short delay between readings
    }
    int distance = -1;
    if (vailadReadings > 0) {
        distance = sumDistance / vailadReadings;
    }
    return distance;
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
    digitalWrite(PMW, setSpeed);
    return setSpeed;
}

// Stop both motors
void stopMotor() {
    setMotorSpeed(L_IN1, L_IN2, L_PWM, 0);
    setMotorSpeed(R_IN1, R_IN2, R_PWM, 0);
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, 0);
}

// Set both motors direction
void setMotor(const int left_speed, const int right_speed) {
    int leftSpeed = constrain(left_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    int rightSpeed = constrain(right_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    //Set left motor direction
    leftSpeed = setMotorSpeed(L_IN1, L_IN2, L_PWM, leftSpeed);
    //Set right motor direction
    rightSpeed = setMotorSpeed(R_IN1, R_IN2, R_PWM, rightSpeed);
    //Limit speed to MOTOR_MAX_SPEED
    Serial.print("Left Motor Speed: ");
    Serial.print(leftSpeed);
    Serial.print("\tRight Motor Speed: ");
    Serial.println(rightSpeed);
}

// Get the error distance from the safe distance
int getErrorDistance(const unsigned int Distance){
    return Distance - SAFE_DISTANCE;
}

// Get different gear based on the error distance
int getDifferentGear(const int errorDistance) {
    int differentGear = (errorDistance * BASED_DIFFERENT_GEAR) / SAFE_DISTANCE;
    if (differentGear > BASED_DIFFERENT_GEAR) {
        differentGear = BASED_DIFFERENT_GEAR;
    }
    return differentGear;
}

// Check if there is an obstacle in front of the ship
bool checkObstacle(const unsigned int frontDistance) {
    if ((frontDistance > 0 && frontDistance < SAFE_DISTANCE)) return true;
    return false;
}

// Move the ship based on the sonar distances
void Move(const unsigned int frontDistance, const unsigned int rightDistance) {
    int errorDistance = getErrorDistance(rightDistance);
    Serial.print("Error Distance: ");
    Serial.print(errorDistance);
    Serial.println(" cm");
    if (abs(errorDistance) <= TOLLRENT_DISTANCE) {
        setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED);
        Serial.println("Go Straight");
    } else {
        int differentGear = getDifferentGear(abs(errorDistance));
        Serial.print("Different Gear: ");
        Serial.println(differentGear);
        if (errorDistance > 0) {
            setMotor(MOTOR_BASED_SPEED + differentGear , MOTOR_BASED_SPEED);
            Serial.println("Turn Left");
        } else {
            setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + differentGear);
            Serial.println("Turn Right");
        }
    }
}

void loop() {
    //get the distance from sonar
    NewPing sonarFront(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance. 
    NewPing sonarRight(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance. 
    unsigned int frontDistance = getSonarDistance(sonarFront);
    unsigned int rightDistance = getSonarDistance(sonarRight);
    Serial.print(frontDistance);
    Serial.print(" cm\t");
    Serial.print("Right Distance: ");
    Serial.print(rightDistance);
    Serial.println(" cm");
    //If the front distance is less than safe distance, need to avoid obstacle
    if (checkObstacle(frontDistance)) {
        Serial.println("Obstacle Detected");
        stopMotor();
        delay(STOP_TIME);
        return ;
    }
    Move(frontDistance, rightDistance);
    stopMotor();
    delay(STOP_TIME);
}