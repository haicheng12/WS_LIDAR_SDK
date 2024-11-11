#ifndef _SCAN_TASK_H_
#define _SCAN_TASK_H_

#include "task.h"

#include "task/scan_task.h"

namespace wr_scan
{
	class ScanTask : public Task
	{
	public:
		ScanTask();
		virtual ~ScanTask();

		bool initial() override;

		void sendScanData(const char *str_data); // 发送获取SN数据

		bool getScanConnect();			// 获取雷达连接信号
		void setScanConnect(bool flag); // 设置雷达连接信号

	private:
		static void *receiveScanFunc(void *arg);

		pthread_t scan_th_;			 // 点云监听和发送线程
		pthread_mutex_t scan_mutex_; // 锁

		int sock_scan_; // udp变量定义
		struct sockaddr_in server_addr_scan_;
		socklen_t addr_size_scan_;

		bool is_scan_connected_; // 雷达启动完成信号

		Udp *scan_udp_task_;

	protected:
		// ROS messages (topics)
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;
	};
}

#endif
