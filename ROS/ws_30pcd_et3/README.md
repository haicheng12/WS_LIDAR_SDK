# ws_30pcd_et3纯固态面阵雷达驱动包

**前提环境**

本代码基于ROS1，有关ROS1的安装教程参考[ROS官网](https://www.ros.org)，
或者[ROS WIKI](https://wiki.ros.org/cn/)

**创建工作空间和编译代码**
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace 

将ws_30pcd_et3这个包放到catkin_ws/src目录，然后编译
cd ~/catkin_ws/
catkin_make

设置环境变量：
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

**连接设备**
```
设置电脑IP地址为192.168.137.100（雷达的IP默认为192.168.137.200，同网段即可），网关255.255.255.0
```

**测试连接**
```
ping 192.168.137.200
```

**启动代码**
```
roslaunch ws_30pcd_et3 scan_frame.launch
```

**服务调用**
```
获取雷达和IMU的数据:
rosrun ws_30pcd_et3 state_client 1
暂停获取雷达和IMU的数据:
rosrun ws_30pcd_et3 state_client 2
获取雷达的SN码:
rosrun ws_30pcd_et3 state_client 3
进入低功耗模式:
rosrun ws_30pcd_et3 state_client 4
恢复正常工作模式:
rosrun ws_30pcd_et3 state_client 5
```

**代码说明**
```
雷达frame_id为ws_scan，topic为/ws_points_raw
imu的frame_id为ws_imu，topic为/ws_imu_raw
```

**时间同步**
```
库文件在/wr_scan_et3/data目录

先安装ptpd，在自己笔记本
sudo apt install libpcap-dev
cd ptpd
make

设置笔记本端为PTP协议的Master，做为时间同步源（每次笔记本开机启动一次即可）
cd ~/ptpd/src
sudo ./ptpd2 -M -i eno1 #这个eno1根据自己笔记本设置，ifconfig查看192.168.137.100的配置
```

**前端界面**
```
推荐使用谷歌浏览器

网页端输入：
http://192.168.137.200:8081

登陆用户名密码：
user1
abc2024
```

