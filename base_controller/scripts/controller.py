#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from base_controller.msg import Speed
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16


class Controller:
    def __init__(self):
        rospy.init_node('base_controller', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('cmd_vel', Twist, self.twist_callback)
        rospy.Subscriber('car_speed', Speed, self.speed_callback)
        self.left_motor_pub = rospy.Publisher(
            'left_motor', Int16, queue_size=1)
        self.right_motor_pub = rospy.Publisher(
            'right_motor', Int16, queue_size=1)

        self.left_vel = 0  # 期望速度
        self.right_vel = 0
        self.left_speed_error = 0  # 偏差
        self.left_speed_error_old = 0  # 上一次偏差
        self.left_speed_error_add = 0  # 偏差积分
        self.left_duty = 0  # 输出占空比
        self.right_speed_error = 0
        self.right_speed_error_old = 0
        self.right_speed_error_add = 0
        self.right_duty = 0

        rospy.spin()

    def twist_callback(self, data):
        self.linear_x = data.linear.x  # 前进
        self.angular_z = data.angular.z  # 旋转

        # 将 Twist 类型的消息转换为左右轮（期望）速度
        diff_vel = self.angular_z*0.02/0.0077  # Arduino 控制周期 20 ms，差速系数 0.0077
        self.left_vel = self.linear_x - diff_vel/2
        self.right_vel = self.linear_x + diff_vel/2

    def speed_callback(self, data):
        rospy.loginfo('Left Speed: %f  Right Speed: %f',
                      data.left_speed, data.right_speed)

        left_Kp = 25
        left_Ki = 0.1
        left_Kd = 0
        self.left_speed_error_old = self.left_speed_error
        self.left_speed_error = self.left_vel - data.left_speed
        self.left_speed_error_add += self.left_speed_error
        if self.left_speed_error_add > 20:  # 积分限幅
            self.left_speed_error_add = 20
        elif self.left_speed_error_add < -20:
            self.left_speed_error_add = -20
        self.left_duty += left_Kp*self.left_speed_error + left_Ki * \
            self.left_speed_error_add + left_Kd * \
            (self.left_speed_error-self.left_speed_error_old)  # 死区

        right_Kp = 25
        right_Ki = 0.1
        right_Kd = 0
        self.right_speed_error_old = self.right_speed_error
        self.right_speed_error = self.right_vel - data.right_speed
        self.right_speed_error_add += self.right_speed_error
        if self.right_speed_error_add > 20:
            self.right_speed_error_add = 20
        elif self.right_speed_error_add < -20:
            self.right_speed_error_add = -20
        self.right_duty += right_Kp*self.right_speed_error + right_Ki*self.right_speed_error_add + \
            right_Kd*(self.right_speed_error - self.right_speed_error_old)

        if self.left_vel == 0 and self.left_speed_error > -0.0001 and self.left_speed_error < 0.0001:
            self.left_duty = 0
        if self.right_vel == 0 and self.right_speed_error > -0.0001 and self.right_speed_error < 0.0001:
            self.right_duty = 0

        # 限幅
        if self.left_duty > 255:
            self.left_duty = 255
        elif self.left_duty < -255:
            self.left_duty = -255
        if self.right_duty > 255:
            self.right_duty = 255
        elif self.right_duty < -255:
            self.right_duty = -255

        self.left_motor_pub.publish(self.left_duty)
        self.right_motor_pub.publish(self.right_duty)

    def shutdown(self):
        rospy.loginfo('Stopping the robot...')
        self.left_motor_pub.publish(0)
        self.right_motor_pub.publish(0)
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        Controller()
    except rospy.ROSInterruptException:
        pass
