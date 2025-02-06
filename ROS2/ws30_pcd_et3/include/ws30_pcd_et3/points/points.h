#ifndef _POINTS_H_
#define _POINTS_H_

#include "../main.h"

#define POINT_DATA_LEN 54000

struct PointData // 点云数据
{
	float x;
	float y;
	float z;
	int intensity;
	int ring;	// 每个点所属的线号，目前分成90块，按照0-89
	float time; // 每个点相对于一帧点云初始时刻扫描的相对时间，一块内的所有点可以付给相同的相对时间
};

namespace wr_scan
{
	class PointCloudPublisher : public rclcpp::Node
	{
	public:
		PointCloudPublisher(std::string name) : Node(name)
		{
			RCLCPP_INFO(this->get_logger(), "%s节点已经启动", name.c_str());

			initUdp("192.168.137.200", 1001); // 写入服务器IP地址，端口号
			sendDataToServer("hello,points"); // 向服务器发送数据

			input_data_cloud_a_ = (PointData *)calloc(POINT_DATA_LEN, sizeof(PointData)); // 开辟空间
			input_data_cloud_b_ = (PointData *)calloc(POINT_DATA_LEN, sizeof(PointData));

			points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_raw", 10); // 创建发布者

			// 接收数据定时器得使用微秒级别，才能完整接收数据
			receive_timer_ = this->create_wall_timer(std::chrono::microseconds(100), std::bind(&PointCloudPublisher::receivePoints, this)); // 创建定时器
			// 发布数据定时器使用毫秒级别
			pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PointCloudPublisher::pubPoints, this)); // 创建定时器
		}

	private:
		void receivePoints() // 接收点云数据
		{
			recvfrom(sock_, (char *)recvData_, sizeof(PCLData), 0, (struct sockaddr *)&server_addr_, &addr_size_); // 接收服务器回应

			if (recvData_->data_type[0] == 0x5A && recvData_->data_type[1] == 0xA5) // 帧头
			{
				count_++; // 接收450次为一帧完整的数据

				if (recvData_->lable == 0x00) // 开始
				{
					if (flag_)
					{
						flag_ = false;
					}
					else
					{
						flag_ = true;
					}
				}

				// 雷达传过来的当前时间
				uint64_t timestamp_data = recvData_->timestamp; // 单位：微秒

				points_time_sec_ = timestamp_data / (1000 * 1000);	// 单位：秒
				points_time_nsec_ = (timestamp_data % 1000) * 1000; // 单位：纳秒

				timestamp_data = timestamp_data * 0.001;

				for (int i = 0; i < 120; i++)
				{
					int index = (count_ - 1) * 120 + i;

					int row = recvData_->row[i];							 // 行 0-149
					int col = recvData_->col[i];							 // 列 0-359
					float point_x = (float)recvData_->point_x[i] / 1000 * 2; // x
					float point_y = (float)recvData_->point_y[i] / 1000 * 2; // y
					float point_z = (float)recvData_->point_z[i] / 1000 * 2; // z
					int intensity = recvData_->intensity[i];

					PointData cloud_temp;
					cloud_temp.x = point_x;
					cloud_temp.y = point_y;
					cloud_temp.z = point_z;
					cloud_temp.intensity = intensity;
					cloud_temp.ring = col / 4;				 // 按照列来区分，列为0-359，等效的线号0-89
					cloud_temp.time = (float)timestamp_data; // 600个点云为一个时间戳

					if (flag_)
					{
						input_data_cloud_a_[index] = cloud_temp;
					}
					else
					{
						input_data_cloud_b_[index] = cloud_temp;
					}
				}

				if (recvData_->lable == 0x02)
				{
					count_ = 0;

					if (flag_)
					{
						use_a_or_b_ = true; // use a
					}
					else
					{
						use_a_or_b_ = false; // use b
					}
					lable_data_ = recvData_->lable;
				}
			}
		}

		void pubPoints() // 发布点云数据
		{
			pcl::PointCloud<pcl::PointXYZI> cloud;
			cloud.width = POINT_DATA_LEN;
			cloud.height = 1;
			cloud.is_dense = false;
			cloud.points.resize(cloud.width * cloud.height);

			if (lable_data_ == 2)
			{
				if (use_a_or_b_)
				{
					// 发布点云a
					for (size_t i = 0; i < cloud.points.size(); ++i)
					{
						cloud.points[i].x = input_data_cloud_a_[i].x;
						cloud.points[i].y = input_data_cloud_a_[i].y;
						cloud.points[i].z = input_data_cloud_a_[i].z;
						cloud.points[i].intensity = input_data_cloud_a_[i].intensity;
					}
					// 转换为 PointCloud2 消息
					sensor_msgs::msg::PointCloud2 msg2;
					pcl::toROSMsg(cloud, msg2);
					msg2.header.frame_id = "ws_scan"; //  设置坐标系  
					msg2.header.stamp.sec = points_time_sec_;
					msg2.header.stamp.nanosec = points_time_nsec_;
					points_publisher_->publish(msg2); // 现在msg2包含了点云数据，可以发布或者处理
				}
				else
				{
					// 发布点云b
					for (size_t i = 0; i < cloud.points.size(); ++i)
					{
						cloud.points[i].x = input_data_cloud_b_[i].x;
						cloud.points[i].y = input_data_cloud_b_[i].y;
						cloud.points[i].z = input_data_cloud_b_[i].z;
						cloud.points[i].intensity = input_data_cloud_b_[i].intensity;
					}
					// 转换为 PointCloud2 消息
					sensor_msgs::msg::PointCloud2 msg2;
					pcl::toROSMsg(cloud, msg2);
					msg2.header.frame_id = "ws_scan"; //  设置坐标系  
					msg2.header.stamp.sec = points_time_sec_;
					msg2.header.stamp.nanosec = points_time_nsec_;
					points_publisher_->publish(msg2); // 现在msg2包含了点云数据，可以发布或者处理
				}
				lable_data_ = 0;
			}
		}

		bool initUdp(const char *ip_data, int host) // 初始化UDP相关配置
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
		}

		rclcpp::TimerBase::SharedPtr receive_timer_;								   // 声明定时器指针
		rclcpp::TimerBase::SharedPtr pub_timer_;									   // 声明定时器指针
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_; // 声明话题发布者指针

		int sock_; // udp变量定义
		struct sockaddr_in server_addr_;
		socklen_t addr_size_;

		PCLData *recvData_ = new PCLData(); // 接收的点云数据定义
		bool flag_ = false;
		int count_ = 0;

		uint32_t points_time_sec_; // 点云时间戳
		uint32_t points_time_nsec_;

		PointData *input_data_cloud_a_ = NULL; // 点云的数据
		PointData *input_data_cloud_b_ = NULL;

		int lable_data_ = 0;	  // 标签
		bool use_a_or_b_ = false; // 标志位
	};
}

#endif
