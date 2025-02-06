#include "../include/ws30_pcd_et3/points/points.h"
#include "../include/ws30_pcd_et3/imu/imu.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 初始化rclcpp

    auto points_node = std::make_shared<wr_scan::PointCloudPublisher>("points_pub"); // 创建对应节点的共享指针对象
    auto imu_node = std::make_shared<wr_scan::ImuPublisher>("imu_pub");              // 创建对应节点的共享指针对象

    // 创建并启动第一个节点的线程
    std::thread node1_thread([&points_node]()
                             { rclcpp::spin(points_node); });

    // 创建并启动第二个节点的线程
    std::thread node2_thread([&imu_node]()
                             { rclcpp::spin(imu_node); });

    // 等待线程完成
    node1_thread.join();
    node2_thread.join();

    rclcpp::shutdown();

    return 0;
}