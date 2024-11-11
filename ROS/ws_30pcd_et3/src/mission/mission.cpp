#include "mission/mission.h"

namespace wr_scan
{
    Mission::Mission() // 构造函数
    {
        initial();

        points_state_pub_ = nh_.advertise<std_msgs::Int32>("/ws_points_state_info", 1); // 发布雷达点云状态
        model_state_pub_ = nh_.advertise<std_msgs::Int32>("/ws_model_state_info", 1);   // 发布雷达模式状态

        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/scan_marker", 1); // 发布雷达方框

        ws_state_service_ = nh_.advertiseService("ws_state_num", &Mission::scanStateServer, this); // 服务调用
    }

    Mission::~Mission() // 析构函数
    {
        delete point_task_; // 释放空间
        delete imu_task_;   // 释放空间
        delete scan_task_;  // 释放空间
    }

    bool Mission::initial()
    {
        point_task_ = new PointsTask(); // 开辟空间
        imu_task_ = new ImuTask();      // 开辟空间
        scan_task_ = new ScanTask();    // 开辟空间

        ws_points_state_ = PointsState::POINTS; // 初始化为点云状态
        ws_model_state_ = ModelState::SN;       // 初始化为获取SN状态

        // 获取点云、IMU和SN码
        point_task_->sendPointsData("hello,points");
        imu_task_->sendImuData("hello,imu");
        scan_task_->sendScanData("hello,sn");

        return true;
    }

    // 状态1：获取点云和IMU数据
    // 状态2：暂停获取点云和IMU数据
    // 状态3：获取SN数据
    // 状态4：进入低功耗模式
    // 状态5：恢复正常工作模式
    bool Mission::scanStateServer(ws_30pcd_et3::state::Request &req,
                                  ws_30pcd_et3::state::Response &res) // 服务调用
    {
        int state_num = req.num;
        if (state_num == 1)
        {
            std::cout << "获取雷达和IMU数据" << std::endl;

            ws_points_state_ = PointsState::POINTS;

            point_task_->sendPointsData("hello,points");
            imu_task_->sendImuData("hello,imu");

            res.sum = state_num;

            return true;
        }
        else if (state_num == 2)
        {
            std::cout << "暂停获取雷达和IMU数据" << std::endl;

            ws_points_state_ = PointsState::STOP;

            point_task_->sendPointsData("stop,points");
            imu_task_->sendImuData("stop,imu");

            res.sum = state_num;

            return true;
        }
        else if (state_num == 3)
        {
            std::cout << "获取雷达身份码" << std::endl;

            ws_model_state_ = ModelState::SN;

            scan_task_->sendScanData("hello,sn");

            res.sum = state_num;

            return true;
        }
        else if (state_num == 4)
        {
            std::cout << "进入低功耗模式" << std::endl;

            ws_model_state_ = ModelState::LOW;

            scan_task_->sendScanData("hello,low");

            res.sum = state_num;

            return true;
        }
        else if (state_num == 5)
        {
            std::cout << "恢复正常工作模式" << std::endl;

            ws_model_state_ = ModelState::WORK;

            scan_task_->sendScanData("hello,work");

            res.sum = state_num;

            return true;
        }

        return true;
    }

    void Mission::pubStateInfo() // 发布雷达各个状态
    {
        // 发布雷达点云状态
        std_msgs::Int32 points_state_info;
        points_state_info.data = ws_points_state_;
        points_state_pub_.publish(points_state_info);

        // 发布雷达模式状态
        std_msgs::Int32 model_state_info;
        model_state_info.data = ws_model_state_;
        model_state_pub_.publish(model_state_info);
    }

    void Mission::pubMarker() // 发布雷达方框
    {
        visualization_msgs::Marker marker;
        // 创建一个新的Marker
        marker.header.frame_id = "ws_scan";
        marker.header.stamp = ros::Time::now();
        marker.ns = "scan_ns";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        // 设置长方体的位置和方向
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // 设置长方体的大小
        marker.scale.x = 0.06;  // x方向的大小
        marker.scale.y = 0.082; // y方向的大小
        marker.scale.z = 0.072; // z方向的大小

        // 设置颜色
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        // 发布Marker
        marker_pub_.publish(marker);

        ros::spinOnce();
    }

    void Mission::run()
    {
        ros::Rate loop_rate(1);
        while (ros::ok())
        {
            pubStateInfo(); // 发布雷达状态
            pubMarker();    // 发布雷达方框

            scan_task_->sendScanData("hello,connect"); // 一直发送握手信号给雷达

            bool is_connected = scan_task_->getScanConnect(); // 获取雷达连接信号
            if (is_connected)
            {
                // 获取点云、IMU和SN码
                point_task_->sendPointsData("hello,points");
                imu_task_->sendImuData("hello,imu");
                scan_task_->sendScanData("hello,sn");

                is_connected = false;                     // 执行一次即可
                scan_task_->setScanConnect(is_connected); // 设置雷达连接信号
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}
