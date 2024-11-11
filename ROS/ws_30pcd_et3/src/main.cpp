#include "mission/mission.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_frame");
    ros::NodeHandle nh;

    wr_scan::Mission *mission = new wr_scan::Mission(); // 开辟空间
    mission->run();

    ros::spin();

    delete mission; // 释放空间

    return 0;
}
