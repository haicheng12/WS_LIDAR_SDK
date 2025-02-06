编译运行代码：
colcon build --packages-select ws30_pcd_et3
source install/setup.bash
ros2 run ws30_pcd_et3 main_node

通过RVIZ显示：
ros2 run rviz2 rviz2
点击RVIZ的左上角File->Open Config->加载ws30_pcd_et3/rviz2/points_imu.rviz
