#include <NewPing.h>

// Set the pin of right motor
#define L_PWM 11
#define L_IN1 7
#define L_IN2 6
// Set the pin of left motor
#define R_PWM 3
#define R_IN3 4
#define R_IN4 5

// Set the front sensor pin
#define FRONT_TRIGGER_PIN 13
#define FRONT_ECHO_PIN 12
// Set the right sensor pin
#define RIGHT_TRIGGER_PIN 10
#define RIGHT_ECHO_PIN 9

//Set the speed of motor
#define MOTOR_BASED_SPEED 180
#define MOTOR_MAX_SPEED 255
#define MOTOR_MIN_SPEED 130
#define BASED_DIFFERENT_GEAR 20
#define LEFT_STRAIGHT_DIFFERENT_GEAR 30
#define RIGHT_STRAIGHT_DIFFERENT_GEAR 10
#define TURNING_DIFFERENT_GEAR 50
#define TURNING_GEAR_BOOST 50

// Heading alignment & correction cadence tuning
#define HEADING_ALIGN_STEP_TIME 60 // in ms
#define HEADING_DERIVATIVE_TOLERANCE 2 // in cm
#define HEADING_MAX_ITERATIONS 3
#define HEADING_ADJUST_OFFSET 20
#define HEADING_DIFFERENT_GEAR 10
#define CORRECTION_COOLDOWN_MS 200 // min spacing between corrections (prevent rapid alternation)
#define CORRECTION_STABILITY_TIME 50 // hold steady after correction to let ship settle

//Set the distance of sensor
#define FRONT_SAFE_DISTANCE 120 // in cm
#define FRONT_STOP_DISTANCE 90 // in cm
#define SAFE_DISTANCE 40 // in cm
#define MAX_DISTANCE 200 // in cm
#define LEFT_TOLERANT_DISTANCE 25 // in cm
#define RIGHT_TOLERANT_DISTANCE 35 // in cm

// Set the sonar reading parameters
#define WAIT_TIME 5 // in ms
#define STRAIGHT_STOP_TIME 80 // in ms
#define ADAPTATION_STOP_TIME 80 // in ms
#define TURNING_STOP_TIME 20 // in ms
#define MAX_TURN_TIME 3000 // in ms
#define READ_ROUNDS 5

unsigned long start;
bool turning = false;
unsigned long lastCorrectionMs = 0;
bool correctionNeeded = false;

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

// Helper: read right distance quickly using same filtering
unsigned int readRightDistance() {
    NewPing sonarRight(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);
    return getSonarDistance(sonarRight);
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

// After a lateral correction, realign bow so the sensor faces perpendicular to shore
void AlignHeading() {
    for (int i = 0; i < HEADING_MAX_ITERATIONS; i++) {
        // Run straight briefly and measure derivative of right distance
        setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + HEADING_DIFFERENT_GEAR);
        unsigned int r1 = readRightDistance();
        delay(HEADING_ALIGN_STEP_TIME);
        unsigned int r2 = readRightDistance();
        int delta = (int)r2 - (int)r1; // >0: bow yawing outward, <0: inward
        Serial.print("[AlignHeading] dRight: ");
        Serial.println(delta);
        if (abs(delta) <= HEADING_DERIVATIVE_TOLERANCE) {
            // heading is sufficiently parallel to shore
            break;
        }
        if (delta > 0) {
            // Right distance increasing -> rotate right a bit to face inward
            setMotor(MOTOR_BASED_SPEED + HEADING_ADJUST_OFFSET, MOTOR_BASED_SPEED);
        } else {
            // Right distance decreasing -> rotate left a bit to face outward
            setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + HEADING_ADJUST_OFFSET);
        }
        delay(HEADING_ALIGN_STEP_TIME);
    }
    lastCorrectionMs = millis();
    correctionNeeded = false;
}

// Move the ship based on the sonar distances
void MoveStraight(const unsigned int rightDistance) {

    int errorDistance = getErrorDistance(rightDistance);
    Serial.print("Error Distance: ");
    Serial.print(errorDistance);
    Serial.println(" cm");
    
    // Check if within acceptable band
    bool withinBand = (errorDistance >= -LEFT_TOLERANT_DISTANCE) && (errorDistance <= RIGHT_TOLERANT_DISTANCE);
    
    if (withinBand) {
        // Already aligned, go straight with slight right bias to maintain course
        setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + BASED_DIFFERENT_GEAR);
        delay(STRAIGHT_STOP_TIME);
        Serial.println("[MoveStraight] Go Straight (aligned)");
        correctionNeeded = false;
    } else {
        // Out of tolerance band - check cooldown to prevent rapid alternation
        unsigned long nowMs = millis();
        unsigned long timeSinceCorrection = nowMs - lastCorrectionMs;
        
        if (timeSinceCorrection < CORRECTION_COOLDOWN_MS && !correctionNeeded) {
            // Still in cooldown period, hold steady without new correction
            setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + BASED_DIFFERENT_GEAR);
            delay(STRAIGHT_STOP_TIME);
            Serial.println("[MoveStraight] Cooldown: hold straight");
            return;
        }
        
        // Apply correction
        int gear = (errorDistance > 0) ? RIGHT_STRAIGHT_DIFFERENT_GEAR : LEFT_STRAIGHT_DIFFERENT_GEAR;
        int differentGear = getDifferentGear(abs(errorDistance), gear, SAFE_DISTANCE);
        Serial.print("Different Gear: ");
        Serial.println(differentGear);
        
        if (errorDistance > 0) {
            // Too far from right shore, turn right (away from shore)
            setMotor(MOTOR_BASED_SPEED + differentGear, MOTOR_BASED_SPEED + BASED_DIFFERENT_GEAR);
            Serial.println("[MoveStraight] Correct Right");
        } else {
            // Too close to left shore, turn left (away from shore)
            Serial.println("[MoveStraight] Correct Left");
            setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + differentGear);
        }
        
        delay(ADAPTATION_STOP_TIME);
        
        // Hold steady after correction to let ship settle
        setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + BASED_DIFFERENT_GEAR);
        delay(CORRECTION_STABILITY_TIME);
        Serial.println("[MoveStraight] Stability hold");
        
        // Mark that a correction was just applied
        correctionNeeded = true;
        lastCorrectionMs = millis();
        
        // Only align heading if error is significant (avoid over-tuning)
        if (abs(errorDistance) > LEFT_TOLERANT_DISTANCE + 15) {
            AlignHeading();
        }
    }
}


// Adaptive left turn that keeps pivoting until front is clear and right distance is reasonable
void TurnLeft(const unsigned int frontDistance) {
    int frontGap = FRONT_SAFE_DISTANCE - (int)frontDistance; // >0 means still too close in front
    int turnBoost = (abs(frontGap) * (MOTOR_MAX_SPEED - MOTOR_BASED_SPEED - TURNING_GEAR_BOOST)) / FRONT_STOP_DISTANCE;
    int leftSpeed = 0;
    if (frontGap < FRONT_STOP_DISTANCE) {
        leftSpeed = MOTOR_MIN_SPEED * (FRONT_STOP_DISTANCE - frontGap) / FRONT_STOP_DISTANCE;
    }
    setMotor(leftSpeed, MOTOR_BASED_SPEED + TURNING_GEAR_BOOST + turnBoost);
    Serial.print("Turning Left. Front Gap: ");
    Serial.print(frontGap);
    Serial.print(" cm, Turn Boost: ");
    Serial.print(turnBoost);
    Serial.print(" cm, Left Speed: ");
    Serial.println(leftSpeed);
    delay(TURNING_STOP_TIME);
    // stopMotor();
}

void Move(const unsigned int frontDistance, const unsigned int rightDistance) {
    bool status = frontDistance >= FRONT_SAFE_DISTANCE;
    if (turning) {
        unsigned long elapsed = millis() - start;
        if (elapsed >= MAX_TURN_TIME) {
            turning = false;
            setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED);
            delay(TURNING_STOP_TIME * 3);
            return ;
        }
        if (status) {
            turning = false;
            return ;
        }
        TurnLeft(frontDistance);
        return ;
    }
    if (!status) {
        // Need to turn left
        turning = true;
        start = millis();
        TurnLeft(frontDistance);
    } else {
        MoveStraight(rightDistance);
    }
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
}
