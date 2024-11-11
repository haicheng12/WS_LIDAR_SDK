本测试机器为ubuntu20，其他系统参考pcl官网（https://pointclouds.org/downloads/#linux）

下载pcl库文件：
sudo apt install libpcl-dev

查看pcl版本：
dpkg -l | grep pcl 本机的是1.10.0

编译代码：
cd ws30_pcd_et3_demo
mkdir build
cd build
make

运行代码：
./scan_frame
