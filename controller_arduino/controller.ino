#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/Int16.h>

#include <base_controller/Speed.h>

// 引脚定义
#define LEFT_PWM1 5
#define LEFT_PWM2 10
#define LEFT_ENCODER_A 3
#define LEFT_ENCODER_B 8
#define RIGHT_PWM1 6
#define RIGHT_PWM2 9
#define RIGHT_ENCODER_A 2
#define RIGHT_ENCODER_B 7

#define pi 3.1415926

ros::NodeHandle nh;
int16_t leftCnt = 0;
int16_t rightCnt = 0;

void LeftMotorControl(const std_msgs::Int16 &cmd_msg) {
  if (cmd_msg.data > 0) {
    digitalWrite(LEFT_PWM1, LOW);
    analogWrite(LEFT_PWM2, cmd_msg.data);
  } else {
    digitalWrite(LEFT_PWM2, LOW);
    analogWrite(LEFT_PWM1, -cmd_msg.data);
  }
}

void RightMotorControl(const std_msgs::Int16 &cmd_msg) {
  if (cmd_msg.data > 0) {
    digitalWrite(RIGHT_PWM1, LOW);
    analogWrite(RIGHT_PWM2, cmd_msg.data);
  } else {
    digitalWrite(RIGHT_PWM2, LOW);
    analogWrite(RIGHT_PWM1, -cmd_msg.data);
  }
}

base_controller::Speed speed_msg;

ros::Subscriber<std_msgs::Int16> subLeftMotor("left_motor", LeftMotorControl);
ros::Subscriber<std_msgs::Int16> subRightMotor("right_motor",
                                               RightMotorControl);
ros::Publisher pubSpeed("car_speed", &speed_msg);

void leftEncoderCount() {
  if (digitalRead(LEFT_ENCODER_B) == HIGH) {
    leftCnt++;
  } else {
    leftCnt--;
  }
}

void rightEncoderCount() {
  if (digitalRead(RIGHT_ENCODER_B) == LOW) {
    rightCnt++;
  } else {
    rightCnt--;
  }
}

void getSpeed() {
  speed_msg.left_speed = leftCnt * pi / 240;
  speed_msg.right_speed = rightCnt * pi / 240;
  leftCnt = 0;
  rightCnt = 0;
}

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(subLeftMotor);
  nh.subscribe(subRightMotor);
  nh.advertise(pubSpeed);

  pinMode(LEFT_PWM1, OUTPUT);
  pinMode(LEFT_PWM2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderCount,
                  RISING);
  pinMode(LEFT_ENCODER_B, INPUT);

  pinMode(RIGHT_PWM1, OUTPUT);
  pinMode(RIGHT_PWM2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderCount,
                  RISING);
  pinMode(RIGHT_ENCODER_B, INPUT);
}

void loop() {
  nh.spinOnce();
  getSpeed();
  pubSpeed.publish(&speed_msg);
  delay(20);
}
