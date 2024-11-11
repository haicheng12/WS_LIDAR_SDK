#ifndef _UDP_H_
#define _UDP_H_

#include <iostream>

#include <stdio.h> //udp相关
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "task/task.h"

namespace wr_scan
{
	// 基类
	class Udp
	{
	public:
		Udp(int host);
		virtual ~Udp();

		bool exit();

		bool initUdp(const char *ip_data, int host);
		void sendDataToServer(const char *str_data);

		int getSock();
		sockaddr_in getServerAddr();
		socklen_t getAddrSize();

	private:
		int sock_; // udp变量定义
		struct sockaddr_in server_addr_;
		socklen_t addr_size_;
	};
}

#endif
