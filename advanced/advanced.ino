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
#define MOTOR_BASED_SPEED 180
#define MOTOR_MAX_SPEED 255
#define MOTOR_MIN_SPEED 130
#define BASED_DIFFERENT_GEAR 30
#define LEFT_STRAIGHT_DIFFERENT_GEAR 40
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

// Set the front sensor pin
#define FRONT_TRIGGER_PIN 13
#define FRONT_ECHO_PIN 12
// Set the right sensor pin
#define RIGHT_TRIGGER_PIN 10
#define RIGHT_ECHO_PIN 9

//Set the distance of sensor
#define FRONT_SAFE_DISTANCE 120 // in cm
#define FRONT_STOP_DISTANCE 90 // in cm
#define SAFE_DISTANCE 40 // in cm
#define MAX_DISTANCE 200 // in cm
#define LEFT_TOLERANT_DISTANCE 25 // in cm
#define RIGHT_TOLERANT_DISTANCE 35 // in cm

#define MAX_TURN_TIME 3000 // in ms

// Set the sonar reading parameters
#define WAIT_TIME 5 // in ms
#define STRAIGHT_STOP_TIME 80 // in ms
#define ADAPTATION_STOP_TIME 80 // in ms
#define TURNING_STOP_TIME 20 // in ms
#define READ_ROUNDS 5

unsigned long start;
bool turning = false;
unsigned long lastCorrectionMs = 0;
bool correctionNeeded = false;

// Drift velocity tracking for smarter correction
int lastRightDistance = -1; // -1 means not initialized
unsigned long lastMeasureTime = 0;
float driftVelocity = 0.0; // cm/s, positive = drifting away from right shore

// Turn angle tracking for precise turning (避免过度转向)
float estimatedTurnAngle = 0.0; // degrees
unsigned int turnStartRightDist = 0;
unsigned int maxRightDistDuringTurn = 0; // peak distance when leaving old wall
int turnPhase = 0; // 0=not turning, 1=leaving wall, 2=in corner, 3=approaching new wall

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

// Helper: read right distance quickly using same filtering
unsigned int readRightDistance() {
    NewPing sonarRight(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);
    return getSonarDistance(sonarRight);
}

// Estimate turn angle based on right distance pattern during turning
float estimateTurnAngleFromDistance(unsigned int currentRight, unsigned int startRight, unsigned int maxRight) {
    // Method: detect turn phases by right distance pattern
    // Phase 1: distance increasing (leaving old wall)
    // Phase 2: distance at max (in corner area)
    // Phase 3: distance decreasing (approaching new wall)
    
    float angle = 0.0;
    
    if (maxRight > startRight + 20) {
        // We've left the old wall significantly
        float phase1Progress = min(1.0, (maxRight - startRight) / 50.0);
        angle += phase1Progress * 45.0; // first 45 degrees
        
        // If now approaching new wall (distance decreasing from peak)
        if (currentRight < maxRight - 10) {
            float phase3Progress = min(1.0, (maxRight - currentRight) / 40.0);
            angle += phase3Progress * 45.0; // additional 45 degrees
        }
    }
    
    return angle;
}

// Check if turn is complete based on multiple criteria
bool isTurnComplete(unsigned int frontDist, unsigned int rightDist, float turnAngle, unsigned long turnTime) {
    bool frontClear = frontDist >= FRONT_SAFE_DISTANCE;
    bool angleAdequate = turnAngle >= 60.0; // turned at least 60 degrees
    bool rightReasonable = (rightDist >= SAFE_DISTANCE - 15) && (rightDist <= SAFE_DISTANCE + 50);
    bool timeout = turnTime >= MAX_TURN_TIME;
    
    // Complete if: (front clear AND angle adequate) OR (front clear AND right aligned) OR timeout
    return (frontClear && angleAdequate) || (frontClear && rightReasonable) || timeout;
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

// Check if there is an obstacle in front of the ship
int checkObstacle(const unsigned int frontDistance) {
    return frontDistance >= FRONT_SAFE_DISTANCE ? 1 : 0; // 1: clear, 0: obstacle
}

// Move the ship based on the sonar distances
void MoveStraight(const unsigned int rightDistance) {
    unsigned long nowMs = millis();
    int errorDistance = getErrorDistance(rightDistance);
    
    // Calculate drift velocity
    if (lastRightDistance >= 0 && lastMeasureTime > 0) {
        unsigned long timeDiff = nowMs - lastMeasureTime;
        if (timeDiff > 0) {
            // Positive drift = moving away from right shore (right distance increasing)
            driftVelocity = ((float)(rightDistance - lastRightDistance) * 1000.0) / timeDiff; // cm/s
        }
    }
    
    // Update tracking variables
    lastRightDistance = rightDistance;
    lastMeasureTime = nowMs;
    
    Serial.print("Error: ");
    Serial.print(errorDistance);
    Serial.print(" cm, Drift: ");
    Serial.print(driftVelocity, 2);
    Serial.println(" cm/s");
    
    // Check if within acceptable band
    bool withinBand = (errorDistance >= -LEFT_TOLERANT_DISTANCE) && (errorDistance <= RIGHT_TOLERANT_DISTANCE);
    
    // Determine if we're moving toward safety zone (self-correcting)
    bool movingTowardSafety = false;
    if (errorDistance > 0 && driftVelocity < -0.5) {
        // Too far, but drifting back toward shore - self-correcting
        movingTowardSafety = true;
    } else if (errorDistance < 0 && driftVelocity > 0.5) {
        // Too close, but drifting away from shore - self-correcting
        movingTowardSafety = true;
    }
    
    if (withinBand) {
        // Within tolerance band
        if (abs(driftVelocity) < 1.0) {
            // Stable, go straight
            setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + BASED_DIFFERENT_GEAR);
            delay(STRAIGHT_STOP_TIME);
            Serial.println("[MoveStraight] Stable - straight");
            correctionNeeded = false;
        } else {
            // Within band but drifting - apply gentle preemptive correction
            int gentleGear = BASED_DIFFERENT_GEAR / 2;
            if (driftVelocity > 1.0) {
                // Drifting away, gently turn right
                setMotor(MOTOR_BASED_SPEED + gentleGear, MOTOR_BASED_SPEED + BASED_DIFFERENT_GEAR);
                Serial.println("[MoveStraight] Preempt drift right");
            } else {
                // Drifting toward shore, gently turn left
                setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + BASED_DIFFERENT_GEAR + gentleGear);
                Serial.println("[MoveStraight] Preempt drift left");
            }
            delay(STRAIGHT_STOP_TIME);
        }
    } else {
        // Out of tolerance band
        unsigned long timeSinceCorrection = nowMs - lastCorrectionMs;
        
        if (movingTowardSafety && timeSinceCorrection < CORRECTION_COOLDOWN_MS * 2) {
            // Self-correcting, don't interfere - let momentum work
            setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + BASED_DIFFERENT_GEAR);
            delay(STRAIGHT_STOP_TIME);
            Serial.println("[MoveStraight] Self-correcting - hold");
            return;
        }
        
        if (timeSinceCorrection < CORRECTION_COOLDOWN_MS && !correctionNeeded) {
            // Cooldown period
            setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + BASED_DIFFERENT_GEAR);
            delay(STRAIGHT_STOP_TIME);
            Serial.println("[MoveStraight] Cooldown");
            return;
        }
        
        // Calculate correction gear with drift velocity consideration
        int gear = (errorDistance > 0) ? RIGHT_STRAIGHT_DIFFERENT_GEAR : LEFT_STRAIGHT_DIFFERENT_GEAR;
        int baseGear = getDifferentGear(abs(errorDistance), gear, SAFE_DISTANCE);
        
        // Adjust gear based on drift velocity (amplify if drifting wrong way, reduce if drifting right way)
        float driftFactor = 1.0;
        if (errorDistance > 0) {
            // Too far from shore
            if (driftVelocity > 1.0) {
                // Drifting further away - increase correction
                driftFactor = 1.3 + min(abs(driftVelocity) / 10.0, 0.5);
            } else if (driftVelocity < -1.0) {
                // Already drifting back - reduce correction
                driftFactor = 0.6;
            }
        } else {
            // Too close to shore
            if (driftVelocity < -1.0) {
                // Drifting closer - increase correction
                driftFactor = 1.3 + min(abs(driftVelocity) / 10.0, 0.5);
            } else if (driftVelocity > 1.0) {
                // Already drifting away - reduce correction
                driftFactor = 0.6;
            }
        }
        
        int differentGear = (int)(baseGear * driftFactor);
        differentGear = constrain(differentGear, 0, gear);
        
        Serial.print("Gear: ");
        Serial.print(differentGear);
        Serial.print(", Factor: ");
        Serial.println(driftFactor, 2);
        
        if (errorDistance > 0) {
            // Too far, turn right
            setMotor(MOTOR_BASED_SPEED + differentGear, MOTOR_BASED_SPEED + BASED_DIFFERENT_GEAR);
            Serial.println("[MoveStraight] Correct Right");
        } else {
            // Too close, turn left
            setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + BASED_DIFFERENT_GEAR + differentGear);
            Serial.println("[MoveStraight] Correct Left");
        }
        
        delay(ADAPTATION_STOP_TIME);
        
        // Shorter stability hold to check results quickly
        setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + BASED_DIFFERENT_GEAR);
        delay(CORRECTION_STABILITY_TIME / 2);
        
        correctionNeeded = true;
        lastCorrectionMs = millis();
        
        // Only align heading if error is large AND drift is significant
        if (abs(errorDistance) > LEFT_TOLERANT_DISTANCE + 20 && abs(driftVelocity) > 2.0) {
            AlignHeading();
        }
    }
}


// Adaptive left turn with angle tracking to prevent over-turning
void TurnLeft(const unsigned int frontDistance, const unsigned int rightDistance) {
    unsigned long elapsed = millis() - start;
    
    // Update turn phase based on right distance pattern
    if (turnPhase == 1 && rightDistance > maxRightDistDuringTurn) {
        maxRightDistDuringTurn = rightDistance; // still leaving wall
    } else if (turnPhase == 1 && rightDistance < maxRightDistDuringTurn - 5) {
        turnPhase = 2; // peaked, now in corner
    } else if (turnPhase == 2 && rightDistance < SAFE_DISTANCE + 30) {
        turnPhase = 3; // approaching new wall
    }
    
    // Estimate turn angle
    estimatedTurnAngle = estimateTurnAngleFromDistance(rightDistance, turnStartRightDist, maxRightDistDuringTurn);
    
    // Add time-based estimation for robustness
    float timeBasedAngle = (elapsed / 1000.0) * 30.0; // assume ~30 deg/sec turn rate
    estimatedTurnAngle = (estimatedTurnAngle + timeBasedAngle) / 2.0; // average both methods
    
    // Calculate turn speed based on front distance and turn progress
    int frontGap = FRONT_SAFE_DISTANCE - (int)frontDistance;
    int turnBoost = (abs(frontGap) * (MOTOR_MAX_SPEED - MOTOR_BASED_SPEED - TURNING_GEAR_BOOST)) / FRONT_STOP_DISTANCE;
    
    // Reduce turn speed as we approach target angle (avoid over-turning)
    if (estimatedTurnAngle > 50.0) {
        float slowdownFactor = max(0.3, 1.0 - (estimatedTurnAngle - 50.0) / 40.0);
        turnBoost = (int)(turnBoost * slowdownFactor);
    }
    
    int leftSpeed = 0;
    if (frontGap < FRONT_STOP_DISTANCE) {
        leftSpeed = MOTOR_MIN_SPEED * (FRONT_STOP_DISTANCE - frontGap) / FRONT_STOP_DISTANCE;
    }
    
    setMotor(leftSpeed, MOTOR_BASED_SPEED + TURNING_GEAR_BOOST + turnBoost);
    
    Serial.print("[Turn] Angle: ");
    Serial.print(estimatedTurnAngle, 1);
    Serial.print("°, Phase: ");
    Serial.print(turnPhase);
    Serial.print(", Front: ");
    Serial.print(frontDistance);
    Serial.print(", Right: ");
    Serial.print(rightDistance);
    Serial.print(", Boost: ");
    Serial.println(turnBoost);
    
    delay(TURNING_STOP_TIME);
}

void Move(const unsigned int frontDistance, const unsigned int rightDistance) {
    bool status = checkObstacle(frontDistance);
    if (turning) {
        unsigned long elapsed = millis() - start;
        
        // Check if turn is complete using intelligent criteria
        if (isTurnComplete(frontDistance, rightDistance, estimatedTurnAngle, elapsed)) {
            turning = false;
            turnPhase = 0;
            estimatedTurnAngle = 0.0;
            Serial.println("[Turn Complete]");
            // Brief straight movement to stabilize
            setMotor(MOTOR_BASED_SPEED, MOTOR_BASED_SPEED + BASED_DIFFERENT_GEAR);
            delay(100);
            return;
        }
        
        TurnLeft(frontDistance, rightDistance);
        return;
    }
    if (!status) {
        // Need to turn left - initialize turn tracking
        turning = true;
        start = millis();
        turnStartRightDist = rightDistance;
        maxRightDistDuringTurn = rightDistance;
        turnPhase = 1; // starting turn
        estimatedTurnAngle = 0.0;
        Serial.println("[Turn Start]");
        TurnLeft(frontDistance, rightDistance);
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
    // MoveStraight(rightDistance);
}
