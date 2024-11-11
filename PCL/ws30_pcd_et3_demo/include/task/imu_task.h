#ifndef _IMU_TASK_H_
#define _IMU_TASK_H_

#include "task.h"

#include "task/imu_task.h"

namespace wr_scan
{
	// 派生类
	class ImuTask : public Task
	{
	public:
		ImuTask();
		virtual ~ImuTask();

		bool initial() override;

		void sendImuData(const char *str_data);

	private:
		bool initImuUdp(); // 初始化udp通讯相关

		static void *sendImuFunc(void *arg);

		pthread_t imu_th_;			// 点云监听和发送线程
		pthread_mutex_t imu_mutex_; // 锁

		uint32_t imu_time_sec_; // IMU时间戳
		uint32_t imu_time_nsec_;

		int sock_imu_; // udp变量定义
		struct sockaddr_in server_addr_imu_;
		socklen_t addr_size_imu_;

		Udp *imu_udp_task_;
	};
}

#endif
