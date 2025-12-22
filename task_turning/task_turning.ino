#include <NewPing.h>

// Set the pin of left motor
#define L_PWM 3
#define L_IN1 7
#define L_IN2 6
// Set the pin of right motor
#define R_PWM 2
#define R_IN3 5
#define R_IN4 4

//Set the speed of motor
#define MOTOR_BASED_SPEED 110
#define MOTOR_MAX_SPEED 255
#define BASED_DIFFERENT_GEAR 30

// Set the front sensor pin
#define FRONT_TRIGGER_PIN 13                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
#define FRONT_ECHO_PIN 12
// Set the right sensor pin
#define RIGHT_TRIGGER_PIN 10
#define RIGHT_ECHO_PIN 9

//Set the distance of sensor
#define FRONT_SAFE_DISTANCE 150 // in cm
#define SAFE_DISTANCE 20 // in cm
#define MAX_DISTANCE 200 // in cm
#define TOLLRENT_DISTANCE 10 // in cm

#define MIN_TURN_TIME 300 // in ms
#define MAX_TURN_TIME 1500 // in ms

// Set the sonar reading parameters
#define WAIT_TIME 100 // in ms
#define STRAIGHT_STOP_TIME 50 // in ms
#define TURNING_STOP_TIME 10 // in ms
#define READ_ROUNDS 5

unsigned long start;
bool turning = false;

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

// Get the distance from sonar
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
    int leftSpeed = constrain(left_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    int rightSpeed = constrain(right_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
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
int checkObstacle(const unsigned int frontDistance, const unsigned int rightDistance) {
    int frontGap = FRONT_SAFE_DISTANCE - (int)frontDistance; // >0 means still too close in front
    int rightError = getErrorDistance(rightDistance);
    bool frontClear = frontDistance >= FRONT_SAFE_DISTANCE;
    bool rightAligned = abs(rightError) <= TOLLRENT_DISTANCE;
    if (!frontClear) return 0; // No obstacle
    if (!rightAligned) return 1; // Obstacle detected
    return 2; // Obstacle detected
}

// Move the ship based on the sonar distances
void MoveStraight(const unsigned int rightDistance) {
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
    stopMotor();
    delay(STRAIGHT_STOP_TIME);
}

// Adaptive left turn that keeps pivoting until front is clear and right distance is reasonable
void TurnLeft(const unsigned int frontDistance, const unsigned int rightDistance) {
    int frontGap = FRONT_SAFE_DISTANCE - (int)frontDistance; // >0 means still too close in front
    int rightError = getErrorDistance(rightDistance);
    int turnBoost = getDifferentGear(abs(rightError));

    if (frontGap > 0) {
        // If front is still blocked, add extra pivot power
        turnBoost = constrain(turnBoost + BASED_DIFFERENT_GEAR, 0, MOTOR_MAX_SPEED - MOTOR_BASED_SPEED);
    }

    setMotor(-MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + turnBoost);
    Serial.print("Turning Left. Front Gap: ");
    Serial.print(frontGap);
    Serial.print(" cm, Right Error: ");
    Serial.print(rightError);
    Serial.println(" cm");
    stopMotor();
    delay(TURNING_STOP_TIME);
}

void Move(const unsigned int frontDistance, const unsigned int rightDistance) {
    int status = checkObstacle(frontDistance, rightDistance);
    if (turning) {
        unsigned long elapsed = millis() - start;
        if (elapsed <= MIN_TURN_TIME) {
            TurnLeft(frontDistance, rightDistance);
            return ;
        }
        if (elapsed >= MAX_TURN_TIME) {
            turning = false;
            return ;
        }
        if (status == 2) {
            turning = false;
            return ;
        }
        TurnLeft(frontDistance, rightDistance);
        return ;
    }
    if (status == 0) {
        MoveStraight(rightDistance);
        return ;
    }
    // Need to turn left
    turning = true;
    start = millis();
    TurnLeft(frontDistance, rightDistance);
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
    Move(frontDistance, rightDistance);
}
