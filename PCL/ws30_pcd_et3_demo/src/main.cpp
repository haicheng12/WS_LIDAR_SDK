#include "mission/mission.h"

int main(int argc, char **argv)
{
    wr_scan::Mission *mission = new wr_scan::Mission(); // 开辟空间
    mission->run();

    delete mission; // 释放空间

    return 0;
}
