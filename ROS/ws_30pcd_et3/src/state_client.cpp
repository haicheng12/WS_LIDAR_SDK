#include "ros/ros.h"
#include "ws_30pcd_et3/state.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_client");
  if (argc != 2)
  {
    std::cout << "usage: state_task_num x, for 1 , 2 ,3 ,4 ,5" << std::endl;
    return -1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ws_30pcd_et3::state>("ws_state_num");

  // 1：获取点云和IMU数据
  // 2：暂停获取点云和IMU数据
  // 3：获取SN数据
  // 4：进入低功耗模式
  // 5：恢复正常工作模式
  ws_30pcd_et3::state srv;
  srv.request.num = atoll(argv[1]);
  if (client.call(srv))
  {
    std::cout << "ws task sum: " << (int)srv.response.sum << std::endl;
  }
  else
  {
    std::cout << "Failed to call ws_state_client" << std::endl;
    return -1;
  }

  return 0;
}
