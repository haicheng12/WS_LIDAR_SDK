#ifndef _POINTS_TASK_H_
#define _POINTS_TASK_H_

#include "task.h"

#include "task/points_task.h"

#define POINT_DATA_LEN 54000

#pragma pack(push)
#pragma pack(1)	 // one byte-aligned
struct PointData // 点云数据
{
	float x;
	float y;
	float z;
	int intensity;
	int ring;	// 每个点所属的线号，目前分成90块，按照0-89
	float time; // 每个点相对于一帧点云初始时刻扫描的相对时间，一块内的所有点可以付给相同的相对时间
};

struct ETPointXYZIRT
{
	PCL_ADD_POINT4D
	PCL_ADD_INTENSITY;
	uint16_t ring;
	float time;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(ETPointXYZIRT,
								  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))
#pragma pack(pop)

namespace wr_scan
{
	// 派生类
	class PointsTask : public Task
	{
	public:
		PointsTask();
		virtual ~PointsTask();

		bool initial() override;

		void sendPointsData(const char *str_data);

	private:
		static void *ListenPointsFunc(void *arg);
		static void *sendPointsFunc(void *arg);

		pthread_t points_listen_th_;		  // 点云监听线程
		pthread_t points_send_th_;			  // 点云发送线程
		pthread_mutex_t points_listen_mutex_; // 锁
		pthread_mutex_t points_send_mutex_;	  // 锁

		int lable_data_ = 0;
		bool use_a_or_b_ = false;

		uint32_t points_time_sec_; // 点云时间戳
		uint32_t points_time_nsec_;

		PointData *input_data_cloud_a_ = NULL; // 点云的数据
		PointData *input_data_cloud_b_ = NULL;

		int sock_points_; // UDP变量定义
		struct sockaddr_in server_addr_points_;
		socklen_t addr_size_points_;

		Udp *points_udp_task_;
	};
}

#endif
