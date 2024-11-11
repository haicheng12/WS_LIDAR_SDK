// 将IMU六轴原始数据转化为roll pitch，客户可根据自身实际情况选择使用与否

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>

#include <iostream>

#define PI 3.1415926

ros::Publisher imu_angle_pub_;

float firstOrderFilter(float angle_m, float gyro_m) // 一阶滤波
{
  float angle = 0.0;
  float K1 = 0.02;

  angle = K1 * angle_m + (1 - K1) * (angle + gyro_m * 0.005);

  return angle;
}

void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  float acc_x = msg->linear_acceleration.x;
  float acc_y = msg->linear_acceleration.y;
  float acc_z = msg->linear_acceleration.z;
  float gyro_x = msg->angular_velocity.x;
  float gyro_y = msg->angular_velocity.y;
  float gyro_z = msg->angular_velocity.z;

  acc_x = atan2(acc_y, acc_z) * 180 / PI;
  gyro_x = gyro_x / 16.4;
  float roll = firstOrderFilter(acc_x, -gyro_x); // 横滚角 roll x轴上的角速度
  // std::cout << "roll = " << roll << std::endl;

  acc_y = atan2(acc_x, acc_z) * 180 / PI;
  gyro_y = gyro_y / 16.4;
  float pitch = firstOrderFilter(acc_y, -gyro_y); // 俯仰角 pitch y轴上的角速度
  // std::cout << "pitch = " << pitch << std::endl;

  float yaw = 0.0; // 六轴的计算yaw不准，据说要磁力计
  // std::cout << "yaw = " << yaw << std::endl;

  geometry_msgs::Vector3 imu_angle_2d;
  imu_angle_2d.x = roll;
  imu_angle_2d.y = pitch;
  imu_angle_2d.z = yaw;

  imu_angle_pub_.publish(imu_angle_2d); // 发布 IMU 角度数据
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calculate_imu");

  ros::NodeHandle nh;

  ros::Subscriber imu_sub = nh.subscribe("/ws_imu_raw", 1, &imuCallback);

  imu_angle_pub_ = nh.advertise<geometry_msgs::Vector3>("/ws_imu_angle", 1);

  ros::spin();

  return 0;
}
