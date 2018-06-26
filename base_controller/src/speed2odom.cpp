#include <base_controller/Speed.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class SubscribeAndPublish {
 public:
  SubscribeAndPublish() {
    x_ = 0.0;
    y_ = 0.0;
    th_ = 0.0;
    vx_ = 0.0;
    vy_ = 0.0;
    vth_ = 0.0;
    current_time_ = ros::Time::now();
    last_time_ = ros::Time::now();

    pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    sub_ = nh_.subscribe("car_speed", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const base_controller::Speed::ConstPtr& input) {
    current_time_ = ros::Time::now();

    /* 速度转换为 odom */
    double delta_speed = input->right_speed - input->left_speed;
    vx_ = (input->left_speed + input->right_speed) / 2;
    vy_ = 0.0;
    vth_ = delta_speed * 0.0077 / 0.02;

    double delta_time = 0.02;  // Arduino 控制周期 20 ms
    double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * delta_time;
    double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * delta_time;
    double delta_th = vth_ * delta_time;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
    /* 发布 tf */
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_.sendTransform(odom_trans);
    /* 发布 odometry 消息 */
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = "odom";  // 位置
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_link";  // 速度
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.angular.z = vth_;

    pub_.publish(odom);

    last_time_ = current_time_;
  }

 private:
  double x_;
  double y_;
  double th_;
  double vx_;
  double vy_;
  double vth_;

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Time current_time_, last_time_;
  tf::TransformBroadcaster odom_broadcaster_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_publisher");

  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
