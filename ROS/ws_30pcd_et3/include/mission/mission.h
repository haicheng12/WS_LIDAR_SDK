#ifndef _MISSION_H_
#define _MISSION_H_

#include <iostream>

#include "ros/ros.h"

#include "task/task.h"
#include "task/imu_task.h"
#include "task/points_task.h"
#include "task/scan_task.h"

#include "udp/udp.h"

#include "ws_30pcd_et3/state.h"

namespace wr_scan
{
	class Mission
	{
	public:
		Mission();
		virtual ~Mission();

		void run();

	private:
		bool initial();		 // 初始化
		void pubStateInfo(); // 发布雷达状态
		void pubMarker();	 // 发布雷达方框

		bool scanStateServer(ws_30pcd_et3::state::Request &req,
							 ws_30pcd_et3::state::Response &res); // 服务调用

		PointsState ws_points_state_; // 雷达点云状态
		ModelState ws_model_state_;	  // 雷达模式状态

		PointsTask *point_task_;
		ImuTask *imu_task_;
		ScanTask *scan_task_;

	protected:
		// ROS messages (topics)
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		ros::Publisher points_state_pub_;
		ros::Publisher model_state_pub_;

		ros::Publisher marker_pub_;

		ros::ServiceServer ws_state_service_;
	};
}

#endif
