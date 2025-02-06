#ifndef _IMU_H_
#define _IMU_H_

#include "../main.h"

namespace wr_scan
{
	class ImuPublisher : public rclcpp::Node
	{
	public:
		ImuPublisher(std::string name) : Node(name)
		{
			RCLCPP_INFO(this->get_logger(), "%s节点已经启动", name.c_str());

			initUdp("192.168.137.200", 1002); // 写入服务器IP地址，端口号
			sendDataToServer("hello,imu");	  // 向服务器发送数据

			imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_raw", 10); // 创建发布者

			timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ImuPublisher::publishImu, this)); // 创建定时器
		}

	private:
		void publishImu() // 发布IMU数据
		{
			recvfrom(sock_, (char *)recvData_, sizeof(ImuData), 0, (struct sockaddr *)&server_addr_, &addr_size_); // 接收数据

			if (recvData_->data_type[0] == 0x1A && recvData_->data_type[1] == 0xA1) // 帧头
			{
				// 时间戳计算
				uint64_t timestamp_data = recvData_->timestamp; // 单位：纳秒

				imu_time_sec_ = timestamp_data / 1000;					// 单位：秒
				imu_time_nsec_ = (timestamp_data % 1000) * 1000 * 1000; // 单位：纳秒

				float acc_x = -recvData_->acc_x; // 根据imu安装位置做一个偏转
				float acc_y = -recvData_->acc_z;
				float acc_z = -recvData_->acc_y;
				float gyro_x = -recvData_->gyro_x;
				float gyro_y = -recvData_->gyro_z;
				float gyro_z = -recvData_->gyro_y;
				// RCLCPP_INFO(this->get_logger(), "原始三轴角加速度:%f,%f,%f", acc_x, acc_y, acc_z);
				// RCLCPP_INFO(this->get_logger(), "原始三轴角速度:%f,%f,%f", gyro_x, gyro_y, gyro_z);

				sensor_msgs::msg::Imu msg; // 创建一个Imu的实例

				msg.header.stamp.sec = imu_time_sec_;	   // 秒
				msg.header.stamp.nanosec = imu_time_nsec_; // 纳秒
				msg.header.frame_id = "ws_imu";

				msg.linear_acceleration.x = acc_x; // 设置线性加速度
				msg.linear_acceleration.y = acc_y;
				msg.linear_acceleration.z = acc_z;
				msg.angular_velocity.x = gyro_x; // 设置角速度
				msg.angular_velocity.y = gyro_y;
				msg.angular_velocity.z = gyro_z;

				imu_publisher_->publish(msg); // 现在msg包含了点云数据，可以发布或者处理
			}
		}

		bool initUdp(const char *ip_data, int host) // 初始化UDP配置相关
		{
			// 创建socket文件描述符
			sock_ = socket(AF_INET, SOCK_DGRAM, 0);

			// 定义服务器地址结构体
			memset(&server_addr_, 0, sizeof(server_addr_));
			server_addr_.sin_family = AF_INET;
			server_addr_.sin_addr.s_addr = inet_addr(ip_data); // 服务器IP
			server_addr_.sin_port = htons(host);			   // 服务器端口

			addr_size_ = sizeof(server_addr_);

			return true;
		}

		void sendDataToServer(const char *str_data) // 向服务器发送消息
		{
			Packet packet_temp;

			strcpy(packet_temp.data, str_data);

			int sent_bytes = sendto(sock_, &packet_temp, sizeof(packet_temp), 0, (struct sockaddr *)&server_addr_, addr_size_);
			// std::cout << "send packet data = " << packet_temp.data << ", successed !!!" << std::endl;
		}

		rclcpp::TimerBase::SharedPtr timer_;								// 声明定时器指针
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_; // 声明话题发布者指针

		int sock_; // udp变量定义
		struct sockaddr_in server_addr_;
		socklen_t addr_size_;

		uint32_t imu_time_sec_; // IMU时间戳
		uint32_t imu_time_nsec_;

		ImuData *recvData_ = new ImuData();
	};
}

#endif
