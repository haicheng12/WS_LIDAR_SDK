#include "udp/udp.h"

namespace wr_scan
{
    Udp::Udp(int host) // 构造函数
    {
        std::string ip_address;
        ros::param::get("~ip_address", ip_address);
        printf("server ip = %s\n", ip_address.c_str());

        char ip_address_temp[256];
        strcpy(ip_address_temp, ip_address.c_str());

        initUdp(ip_address_temp, host);
    }

    Udp::~Udp() // 析构函数
    {
        exit();
    }

    bool Udp::initUdp(const char *ip_data, int host)
    {
        // 创建socket文件描述符
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);

        // 定义服务器地址结构体
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_addr.s_addr = inet_addr(ip_data); // 服务器IP
        server_addr_.sin_port = htons(host);               // 服务器端口

        addr_size_ = sizeof(server_addr_);

        return true;
    }

    bool Udp::exit()
    {
        close(sock_); // 关闭套接字
        return true;
    }

    void Udp::sendDataToServer(const char *str_data) // 向服务器发送消息
    {
        Packet packet_temp;

        strcpy(packet_temp.data, str_data);

        int sent_bytes = sendto(sock_, &packet_temp, sizeof(packet_temp), 0, (struct sockaddr *)&server_addr_, addr_size_);
        // std::cout << "send packet data = " << packet_temp.data << ", successed !!!" << std::endl;
    }

    int Udp::getSock()
    {
        return sock_;
    }

    sockaddr_in Udp::getServerAddr()
    {
        return server_addr_;
    }

    socklen_t Udp::getAddrSize()
    {
        return addr_size_;
    }
}
