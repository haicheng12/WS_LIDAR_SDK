#include "task/scan_task.h"

namespace wr_scan
{
    ScanTask::ScanTask() // 构造函数
    {
        initial();
    }

    ScanTask::~ScanTask() // 析构函数
    {
        close(sock_scan_); // 关闭套接字

        delete scan_udp_task_; // 释放空间
    }

    bool ScanTask::initial()
    {
        scan_udp_task_ = new Udp(1003); // 开辟空间
        sock_scan_ = scan_udp_task_->getSock();
        server_addr_scan_ = scan_udp_task_->getServerAddr();
        addr_size_scan_ = scan_udp_task_->getAddrSize();

        // 创建线程
        pthread_mutex_init(&scan_mutex_, NULL);
        pthread_mutex_lock(&scan_mutex_);
        pthread_create(&scan_th_, NULL, &ScanTask::sendScanFunc, this);
        pthread_mutex_unlock(&scan_mutex_);

        return true;
    }

    void ScanTask::sendScanData(const char *str_data)
    {
        scan_udp_task_->sendDataToServer(str_data);
    }

    bool ScanTask::getScanConnect() // 获取雷达连接信号
    {
        return is_scan_connected_;
    }

    void ScanTask::setScanConnect(bool flag) // 设置雷达连接信号
    {
        is_scan_connected_ = flag;
    }

    void *ScanTask::sendScanFunc(void *arg)
    {
        ScanTask *obj = static_cast<ScanTask *>(arg);

        ScanData *recvData = new ScanData();

        while (1)
        {
            pthread_mutex_lock(&obj->scan_mutex_); // 锁定互斥锁

            // std::cout << "test scan" << std::endl;
            recvfrom(obj->sock_scan_, (char *)recvData, sizeof(ScanData), 0, (struct sockaddr *)&obj->server_addr_scan_, &obj->addr_size_scan_); // 接收数据

            if (recvData->data_type[0] == 0x2A && recvData->data_type[1] == 0xA2) // 帧头
            {
                std::cout << "雷达SN码: " << recvData->sn_data << std::endl;
            }

            if (recvData->data_type[0] == 0x3A && recvData->data_type[1] == 0xA3) // 帧头
            {
                std::cout << "雷达启动完成信号: " << recvData->is_connected << std::endl;
                obj->is_scan_connected_ = recvData->is_connected;
            }

            pthread_mutex_unlock(&obj->scan_mutex_); // 解除互斥锁

            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1毫秒
        }

        pthread_mutex_destroy(&obj->scan_mutex_); // 销毁互斥锁

        return nullptr;
    }
}