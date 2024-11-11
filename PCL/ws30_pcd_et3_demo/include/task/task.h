#ifndef _TASK_H_
#define _TASK_H_

#include <cstdlib>

#include <iostream> //C++相关
#include <cstring>
#include <thread>
#include <pthread.h>
#include <unistd.h>

#include <chrono>
#include <vector>

#include <stdio.h> //udp相关
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <pcl/point_cloud.h> //pcl相关
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

#include <mutex> //锁

#include "udp/udp.h"

// 分为两个状态机:点云获取和暂停点云获取，获取身份码和高低功耗模式
enum PointsState : uint8_t // 雷达点云状态
{
	POINTS, // 获取雷达的点云和IMU数据
	STOP	// 暂停获取雷达的点云和IMU数据
};

enum ModelState : uint8_t // 雷达模式状态
{
	FREE, // 空闲
	SN,	  // 获取身份码
	LOW,  // 低功耗模式
	WORK  // 恢复正常工作模式
};

#pragma pack(push)
#pragma pack(1) // one byte-aligned
struct Packet
{
	char data[256];
};

struct PCLData // UDP接收的PCL数据
{
	uint8_t data_type[2];	// 5A A5
	uint64_t timestamp;		// 时间戳，单位：毫秒
	uint8_t lable;			// 标签
	uint16_t row[100];		// 行
	uint16_t col[100];		// 列
	uint8_t intensity[100]; // 回波强度
	int16_t point_x[100];	// 点云x坐标
	int16_t point_y[100];	// 点云y坐标
	int16_t point_z[100];	// 点云z坐标
};

struct ImuData // UDP接收的IMU数据
{
	uint8_t data_type[2]; // 1A A1
	uint64_t timestamp;	  // 时间戳，单位：毫秒
	float gyro_x;		  // x角速度，单位：rad/s
	float gyro_y;		  // y角速度，单位：rad/s
	float gyro_z;		  // z角速度，单位：rad/s
	float acc_x;		  // x加速度，单位：g
	float acc_y;		  // y加速度，单位：g
	float acc_z;		  // z加速度，单位：g
};

struct ScanData // UDP接收的雷达其他数据
{
	uint8_t data_type[2];
	char sn_data[64];  // 雷达身份码 帧头为2A A2
	bool is_connected; // 雷达启动完成信号 帧头为3A A3
};
#pragma pack(pop)

namespace wr_scan
{
	// 基类
	class Task
	{
	public:
		Task() {};
		virtual ~Task() {};

		virtual bool initial() { return true; };

	private:
	};
}

#endif
