#include "task/imu_task.h"

namespace wr_scan
{
    ImuTask::ImuTask() // 构造函数
    {
        initial();
    }

    ImuTask::~ImuTask() // 析构函数
    {
        close(sock_imu_); // 关闭套接字

        delete imu_udp_task_; // 释放空间
    }

    bool ImuTask::initial()
    {
        // ROS_INFO("初始化");
        imu_udp_task_ = new Udp(1002); // 开辟空间
        sock_imu_ = imu_udp_task_->getSock();
        server_addr_imu_ = imu_udp_task_->getServerAddr();
        addr_size_imu_ = imu_udp_task_->getAddrSize();

        // 创建线程
        pthread_mutex_init(&imu_mutex_, NULL);
        pthread_mutex_lock(&imu_mutex_);
        pthread_create(&imu_th_, NULL, &ImuTask::sendImuFunc, this);
        pthread_mutex_unlock(&imu_mutex_);

        return true;
    }

    void ImuTask::sendImuData(const char *str_data)
    {
        imu_udp_task_->sendDataToServer(str_data);
    }

    void *ImuTask::sendImuFunc(void *arg)
    {
        ImuTask *obj = static_cast<ImuTask *>(arg);

        ImuData *recvData = new ImuData();

        while (1)
        {
            pthread_mutex_lock(&obj->imu_mutex_); // 锁定互斥锁

            // std::cout << "test imu" << std::endl;
            recvfrom(obj->sock_imu_, (char *)recvData, sizeof(ImuData), 0, (struct sockaddr *)&obj->server_addr_imu_, &obj->addr_size_imu_); // 接收数据

            if (recvData->data_type[0] == 0x1A && recvData->data_type[1] == 0xA1) // 帧头
            {
                // 时间戳计算
                uint64_t timestamp_data = recvData->timestamp; // 单位：毫秒
                // std::cout << "timestamp_data = " << timestamp_data << std::endl;

                obj->imu_time_sec_ = timestamp_data / 1000; // 单位：秒
                // std::cout << " imu time_sec_ = " << time_sec_;
                obj->imu_time_nsec_ = (timestamp_data % 1000) * 1000 * 1000; // 单位：纳秒
                // std::cout << " time_nsec_ = " << time_nsec_ << std::endl;

                float acc_x = -recvData->acc_x; // 根据imu安装位置做一个偏转
                float acc_y = -recvData->acc_z;
                float acc_z = -recvData->acc_y;
                float gyro_x = -recvData->gyro_x;
                float gyro_y = -recvData->gyro_z;
                float gyro_z = -recvData->gyro_y;
                // std::cout << "原始三轴角加速度：[ax, ay, az]= [" << acc_x << ", " << acc_y << ", " << acc_z << "]" << std::endl;
                // std::cout << "原始三轴角速度：[gx, gy, gz] = [" << gyro_x << ", " << gyro_y << ", " << gyro_z << "]" << std::endl;
                // std::cout << acc_x << ", " << acc_y << ", " << acc_z << ", " << gyro_x << ", " << gyro_y << ", " << gyro_z << std::endl;

                // std::cout << "发布imu数据" << std::endl;

                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            pthread_mutex_unlock(&obj->imu_mutex_); // 解除互斥锁
        }

        pthread_mutex_destroy(&obj->imu_mutex_); // 销毁互斥锁

        return nullptr;
    }

}