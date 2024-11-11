#ifndef _MISSION_H_
#define _MISSION_H_

#include <iostream>

#include "task/task.h"
#include "task/imu_task.h"
#include "task/points_task.h"
#include "task/scan_task.h"

#include "udp/udp.h"

namespace wr_scan
{
	class Mission
	{
	public:
		Mission();
		virtual ~Mission();

		void run();

	private:
		bool initial(); // 初始化

		PointsState ws_points_state_; // 雷达点云状态
		ModelState ws_model_state_;	  // 雷达模式状态

		PointsTask *point_task_;
		ImuTask *imu_task_;
		ScanTask *scan_task_;

	protected:
	};
}

#endif
