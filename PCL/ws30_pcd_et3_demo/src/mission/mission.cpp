#include "mission/mission.h"

namespace wr_scan
{
    Mission::Mission() // 构造函数
    {
        initial();
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

    void Mission::run()
    {
        while (1)
        {
            // code
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

            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 1秒
        }
    }
}
