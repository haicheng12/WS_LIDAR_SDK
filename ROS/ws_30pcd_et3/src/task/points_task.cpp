#include "task/points_task.h"

namespace wr_scan
{
    PointsTask::PointsTask() // 构造函数
    {
        initial();
    }

    PointsTask::~PointsTask() // 析构函数
    {
        close(sock_points_); // 关闭套接字

        free(input_data_cloud_a_); // 释放指针
        free(input_data_cloud_b_); // 释放指针

        delete points_udp_task_; // 释放空间
    }

    bool PointsTask::initial()
    {
        points_udp_task_ = new Udp(1001); // 开辟空间
        sock_points_ = points_udp_task_->getSock();
        server_addr_points_ = points_udp_task_->getServerAddr();
        addr_size_points_ = points_udp_task_->getAddrSize();

        input_data_cloud_a_ = (PointData *)calloc(POINT_DATA_LEN, sizeof(PointData)); // 开辟空间
        input_data_cloud_b_ = (PointData *)calloc(POINT_DATA_LEN, sizeof(PointData));

        // 创建线程
        pthread_mutex_init(&points_listen_mutex_, NULL);
        pthread_mutex_lock(&points_listen_mutex_);
        pthread_create(&points_listen_th_, NULL, &PointsTask::ListenPointsFunc, this);
        pthread_mutex_unlock(&points_listen_mutex_);

        pthread_mutex_init(&points_send_mutex_, NULL);
        pthread_mutex_lock(&points_send_mutex_);
        pthread_create(&points_send_th_, NULL, &PointsTask::sendPointsFunc, this);
        pthread_mutex_unlock(&points_send_mutex_);

        return true;
    }

    void PointsTask::sendPointsData(const char *str_data)
    {
        points_udp_task_->sendDataToServer(str_data);
    }

    void *PointsTask::ListenPointsFunc(void *arg)
    {
        PointsTask *obj = static_cast<PointsTask *>(arg);

        PCLData *recvData = new PCLData();

        bool flag = false;
        int count = 0;
        while (ros::ok())
        {
            pthread_mutex_lock(&obj->points_listen_mutex_); // 锁定互斥锁
            // 接收服务器回应
            recvfrom(obj->sock_points_, (char *)recvData, sizeof(PCLData), 0, (struct sockaddr *)&obj->server_addr_points_, &obj->addr_size_points_);

            if (recvData->data_type[0] == 0x5A && recvData->data_type[1] == 0xA5) // 帧头
            {
                count++; // 接收540次为一帧完整的数据
                // std::cout << "count " << count << std::endl;//1-540

                if (recvData->lable == 0x00) // 开始
                {
                    if (flag)
                    {
                        flag = false;
                    }
                    else
                    {
                        flag = true;
                    }
                }

                // 雷达传过来的当前时间
                uint64_t timestamp_data = recvData->timestamp; // 单位：毫秒
                // std::cout << "point timestamp_data = " << timestamp_data << std::endl;

                obj->points_time_sec_ = timestamp_data / 1000; // 单位：秒
                // std::cout << " point points_time_sec_ = " << points_time_sec_;
                obj->points_time_nsec_ = (timestamp_data % 1000) * 1000 * 1000; // 单位：纳秒
                // std::cout << " points_time_nsec_ = " << points_time_nsec_ << std::endl;

                for (int i = 0; i < 100; i++)
                {
                    int index = (count - 1) * 100 + i;

                    int row = recvData->row[i]; // 行 0-149
                    int col = recvData->col[i]; // 列 0-359
                    // std::cout << " row = " << row << " col = " << col << std::endl;
                    float point_x = (float)recvData->point_x[i] / 1000 * 2; // x
                    if (point_x < 0)                                        // 去掉负值
                    {
                        double b = 0; // 无穷小
                        double s = 1 / b;
                        point_x = s;
                    }
                    float point_y = (float)recvData->point_y[i] / 1000 * 2; // y
                    float point_z = (float)recvData->point_z[i] / 1000 * 2; // z
                    // printf("点云的坐标数据 = %f %f %f \n", point_x, point_y, point_z);
                    int intensity = recvData->intensity[i];
                    // std::cout << "回波强度 intensity = " << intensity << std::endl;

                    PointData cloud_temp;
                    cloud_temp.x = point_x;
                    cloud_temp.y = point_y;
                    cloud_temp.z = point_z;
                    cloud_temp.intensity = intensity;
                    cloud_temp.ring = col / 4;               // 按照列来区分，列为0-359，等效的线号0-89
                    cloud_temp.time = (float)timestamp_data; // 600个点云为一个时间戳

                    if (flag)
                    {
                        obj->input_data_cloud_a_[index] = cloud_temp;
                    }
                    else
                    {
                        obj->input_data_cloud_b_[index] = cloud_temp;
                    }
                }

                if (recvData->lable == 0x02)
                {
                    count = 0;
                    if (flag)
                    {
                        obj->use_a_or_b_ = true; // use a
                    }
                    else
                    {
                        obj->use_a_or_b_ = false; // use b
                    }

                    obj->lable_data_ = recvData->lable;
                }
            }

            pthread_mutex_unlock(&obj->points_listen_mutex_); // 解除互斥锁
        }
        delete recvData;

        pthread_mutex_destroy(&obj->points_listen_mutex_); // 销毁互斥锁

        return nullptr;
    }

    void *PointsTask::sendPointsFunc(void *arg)
    {
        PointsTask *obj = static_cast<PointsTask *>(arg);

        ros::NodeHandle nh;
        ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/ws_points_raw", 1);

        pcl::PointCloud<ETPointXYZIRT>::Ptr cloud(new pcl::PointCloud<ETPointXYZIRT>); // 创建一个点云指针

        cloud->width = POINT_DATA_LEN; // 填充点云数据
        cloud->height = 1;
        cloud->points.resize(cloud->width * cloud->height);

        while (ros::ok())
        {
            pthread_mutex_lock(&obj->points_send_mutex_); // 锁定互斥锁

            if (obj->lable_data_ == 2)
            {
                if (obj->use_a_or_b_)
                {
                    // 发布点云a
                    for (int i = 0; i < POINT_DATA_LEN; i++)
                    {
                        cloud->points[i].x = obj->input_data_cloud_a_[i].x;
                        cloud->points[i].y = obj->input_data_cloud_a_[i].y;
                        cloud->points[i].z = obj->input_data_cloud_a_[i].z;
                        cloud->points[i].intensity = obj->input_data_cloud_a_[i].intensity;
                        cloud->points[i].ring = obj->input_data_cloud_a_[i].ring;
                        cloud->points[i].time = obj->input_data_cloud_a_[i].time;
                    }
                    sensor_msgs::PointCloud2 output;
                    pcl::toROSMsg(*cloud, output); // 转化为ROS数据
                    output.header.frame_id = "ws_scan";
                    // output.header.stamp = ros::Time::now();
                    output.header.stamp.sec = obj->points_time_sec_;   // 秒
                    output.header.stamp.nsec = obj->points_time_nsec_; // 纳秒
                    pcl_pub.publish(output);
                    ros::spinOnce();
                }
                else
                {
                    // 发布点云b
                    for (int i = 0; i < POINT_DATA_LEN; i++)
                    {
                        cloud->points[i].x = obj->input_data_cloud_b_[i].x;
                        cloud->points[i].y = obj->input_data_cloud_b_[i].y;
                        cloud->points[i].z = obj->input_data_cloud_b_[i].z;
                        cloud->points[i].intensity = obj->input_data_cloud_b_[i].intensity;
                        cloud->points[i].ring = obj->input_data_cloud_b_[i].ring;
                        cloud->points[i].time = obj->input_data_cloud_b_[i].time;
                    }
                    sensor_msgs::PointCloud2 output;
                    pcl::toROSMsg(*cloud, output); // 转化为ROS数据
                    output.header.frame_id = "ws_scan";
                    // output.header.stamp = ros::Time::now();
                    output.header.stamp.sec = obj->points_time_sec_;   // 秒
                    output.header.stamp.nsec = obj->points_time_nsec_; // 纳秒
                    pcl_pub.publish(output);
                    ros::spinOnce();
                }
                obj->lable_data_ = 0;
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1毫秒
            }
            pthread_mutex_unlock(&obj->points_send_mutex_); // 解除互斥锁
        }

        pthread_mutex_destroy(&obj->points_send_mutex_); // 销毁互斥锁

        return nullptr;
    }
}