#include <NewPing.h>

// Set the pin of right motor
#define L_PWM 7
#define L_IN1 6
#define L_IN2 5
// Set the pin of left motor
#define R_PWM 2
#define R_IN3 3
#define R_IN4 4

//Set the speed of motor
#define MOTOR_BASED_SPEED 130
#define MOTOR_MAX_SPEED 255
#define BASED_DIFFERENT_GEAR 0
#define STRAIGHT_DIFFERENT_GEAR 80
#define TURNING_DIFFERENT_GEAR 50
#define TURNING_GEAR_BOOST 30

// Set the front sensor pin
#define FRONT_TRIGGER_PIN 13                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
#define FRONT_ECHO_PIN 12
// Set the right sensor pin
#define RIGHT_TRIGGER_PIN 10
#define RIGHT_ECHO_PIN 9

//Set the distance of sensor
#define FRONT_SAFE_DISTANCE 70 // in cm
#define SAFE_DISTANCE 20 // in cm
#define MAX_DISTANCE 200 // in cm
#define LEFT_TOLERANT_DISTANCE 15 // in cm
#define RIGHT_TOLERANT_DISTANCE 15 // in cm

#define MIN_TURN_TIME 10 // in ms
#define MAX_TURN_TIME 200 // in ms

// Set the sonar reading parameters
#define WAIT_TIME 5 // in ms
#define STRAIGHT_STOP_TIME 200 // in ms
#define ADAPTATION_STOP_TIME 5 // in ms
#define TURNING_STOP_TIME 5 // in ms
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
    Serial.println("Task Turning Start");
}

// Get the distance from sonar using median filter
unsigned int getSonarDistance(NewPing &sonar) {
    unsigned int readings[READ_ROUNDS];
    int validReadings = 0;
    
    // Collect valid readings
    for (int i = 0; i < READ_ROUNDS; i++) {
        unsigned int distance = sonar.ping_cm();
        if (distance > 0) {
            readings[validReadings] = distance;
            validReadings++;
        }
        delay(WAIT_TIME); //Short delay between readings
    }
    
    // Return MAX_DISTANCE if no valid readings
    if (validReadings == 0) {
        return MAX_DISTANCE;
    }
    
    // Sort readings to find median (simple bubble sort for small array)
    for (int i = 0; i < validReadings - 1; i++) {
        for (int j = 0; j < validReadings - i - 1; j++) {
            if (readings[j] > readings[j + 1]) {
                unsigned int temp = readings[j];
                readings[j] = readings[j + 1];
                readings[j + 1] = temp;
            }
        }
    }
    
    // Get median value
    unsigned int median;
    if (validReadings % 2 == 0) {
        median = (readings[validReadings / 2 - 1] + readings[validReadings / 2]) / 2;
    } else {
        median = readings[validReadings / 2];
    }
    
    // If median is 0, return MAX_DISTANCE
    if (median == 0) {
        median = MAX_DISTANCE;
    }
    
    return median;
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
int getDifferentGear(const int errorDistance, const int differentGear, const int safeDistance) {
    int calculatedGear = (errorDistance * differentGear) / safeDistance;
    if (calculatedGear > differentGear) {
        calculatedGear = differentGear;
    }
    return calculatedGear;
}

// Check if there is an obstacle in front of the ship
int checkObstacle(const unsigned int frontDistance, const unsigned int rightDistance) {
    int rightError = getErrorDistance(rightDistance);
    bool frontClear = frontDistance >= FRONT_SAFE_DISTANCE;
    bool rightAligned =  (rightError <= -LEFT_TOLERANT_DISTANCE) || (rightError >= RIGHT_TOLERANT_DISTANCE);
    if (!frontClear) return 0; // No obstacle
    if (rightAligned) return 1; // Obstacle detected
    return 2; // Obstacle detected
}

// Move the ship based on the sonar distances
void MoveStraight(const unsigned int rightDistance) {
    int errorDistance = getErrorDistance(rightDistance);
    Serial.print("Error Distance: ");
    Serial.print(errorDistance);
    Serial.println(" cm");
    if ((errorDistance >= -LEFT_TOLERANT_DISTANCE) && (errorDistance <= RIGHT_TOLERANT_DISTANCE)) {
        setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED);
        delay(STRAIGHT_STOP_TIME);
        Serial.println("Go Straight");
    } else {
        int differentGear = getDifferentGear(abs(errorDistance), STRAIGHT_DIFFERENT_GEAR, SAFE_DISTANCE);
        Serial.print("Different Gear: ");
        Serial.println(differentGear);
        if (errorDistance > 0) {
            setMotor(MOTOR_BASED_SPEED + differentGear , MOTOR_BASED_SPEED);
            Serial.println("Turn Left");
        } else {
            setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + differentGear);
            Serial.println("Turn Right");
        }
        delay(ADAPTATION_STOP_TIME);
        // stopMotor();
    }
}


// Adaptive left turn that keeps pivoting until front is clear and right distance is reasonable
void TurnLeft(const unsigned int frontDistance, const unsigned int rightDistance) {
    int frontGap = FRONT_SAFE_DISTANCE - (int)frontDistance; // >0 means still too close in front
    int turnBoost = getDifferentGear(abs(frontGap), TURNING_DIFFERENT_GEAR, FRONT_SAFE_DISTANCE);
    setMotor(0, MOTOR_BASED_SPEED + TURNING_GEAR_BOOST + turnBoost);
    Serial.print("Turning Left. Front Gap: ");
    Serial.print(frontGap);
    Serial.print(" cm, Turn Boost: ");
    Serial.print(turnBoost);
    Serial.println(" cm");
    delay(TURNING_STOP_TIME);
    // stopMotor();
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
        if (status) {
            turning = false;
            return ;
        }
        TurnLeft(frontDistance, rightDistance);
        return ;
    }
    if (status) {
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
    Serial.print("Front Distance: ");
    Serial.print(frontDistance);
    Serial.print(" cm\t");
    Serial.print("Right Distance: ");
    Serial.print(rightDistance);
    Serial.println(" cm");
    Move(frontDistance, rightDistance);
    // MoveStraight(rightDistance);
}
