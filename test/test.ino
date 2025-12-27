//电机测试程序
#include <NewPing.h>

//left motor
#define A_PWM 11
#define A_IN1 7
#define A_IN2 6

//right motor
#define B_PWM 3
#define B_IN1 4
#define B_IN2 5


//电机停止
void stopMotors() {
  // 左电机停止
  digitalWrite(A_IN1, LOW);
  digitalWrite(A_IN2, LOW);
  analogWrite(A_PWM, 0);
  
  // 右电机停止
  digitalWrite(B_IN1, LOW);
  digitalWrite(B_IN2, LOW);
  analogWrite(B_PWM, 0);
}

// 设置电机速度
void setMotors(int leftSpeed, int rightSpeed) {
  // 限制速度范围
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // 左电机控制
  if (leftSpeed < 0) {
    // 正转
    digitalWrite(A_IN1, HIGH);
    digitalWrite(A_IN2, LOW);
  } else if (leftSpeed > 0) {
    // 反转
    digitalWrite(A_IN1, LOW);
    digitalWrite(A_IN2, HIGH);
  } else {
    // 停止
    digitalWrite(A_IN1, LOW);
    digitalWrite(A_IN2, LOW);
  }
  analogWrite(A_PWM, abs(leftSpeed)); // 设置速度
  
  // 右电机控制
  if (rightSpeed < 0) {
    // 正转
    digitalWrite(B_IN1, HIGH);
    digitalWrite(B_IN2, LOW);
  } else if (rightSpeed > 0) {
    // 反转
    digitalWrite(B_IN1, LOW);
    digitalWrite(B_IN2, HIGH);
  } else {
    // 停止
    digitalWrite(B_IN1, LOW);
    digitalWrite(B_IN2, LOW);
  }
  analogWrite(B_PWM, abs(rightSpeed)); // 设置速度
}

void setup() {
  // put your setup code here, to run once:
  pinMode(A_PWM, OUTPUT);
  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);

  pinMode(B_PWM, OUTPUT);
  pinMode(B_IN1, OUTPUT);
  pinMode(B_IN2, OUTPUT);

  stopMotors();
  Serial.begin(9600);
  Serial.println("Diamond motor test Ready");
}



void loop() {
  // put your main code here, to run repeatedly:
  // for (int speed = 150; speed <= 255; speed++) {
  //   Serial.print("Setting motors to speed: ");
  //   Serial.println(speed);
  //   setMotors(spe;;;;;;;;;;...................../ed, speed);
  //   delay(200);
  // }
  setMotors(0, 255);
  // stopMotors();
}

