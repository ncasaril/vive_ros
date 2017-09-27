#include <errno.h>
#include "UDPBroadcast.h"
#include <ros/ros.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

UDPBroadcast::UDPBroadcast(int port) 
    : _port(port)
{
    _s = socket(AF_INET, SOCK_DGRAM, 0);
    if ((_s < 0)
        ||(SetSocketBlockingEnabled(_s, false)     != B_NO_ERROR)
        ||(BindUDPSocketToBroadcast(_s, port)      != B_NO_ERROR)
        ||(SetUDPSocketBroadcastEnabled(_s, true)  != B_NO_ERROR)
        //||(SetUDPSocketTargetToBroadcast(_s, port) != B_NO_ERROR)
        )
    {
        ROS_ERROR("UDPBroadcast::Error setting up UDP broadcast Socket on port %d!\n", port);
    }
    
    ROS_INFO("UDPBroadcast running on port %d\n", _port);
}

size_t UDPBroadcast::listen(uint8_t* buffer, size_t buffer_size, int timeout_us) const
{
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(_s, &read_set);

    struct timeval timeout;
    
    if (timeout_us > 0)
    {
        timeout.tv_sec = timeout_us / 1000000;
        timeout.tv_usec = (timeout_us % 1000000);
    }

    if (select(_s+1, &read_set, NULL, NULL, 
               (timeout_us > 0) ? &timeout : 0) < 0) 
    {
        char errmsg[256];
        char *err = strerror_r(errno, errmsg, sizeof(errmsg));
        ROS_ERROR("UDPBroadcast::listen: select failed: %s\n", err);
    }

    size_t bytes_read = 0;
    if (FD_ISSET(_s, &read_set))
    {
        bytes_read = recv(_s, buffer, buffer_size, 0);
    }
    return bytes_read;
}

string UDPBroadcast::listen(int timeout_us) const
{
    uint8_t text[8192] = "";

    int bytes_read = listen(text, sizeof(text) - 1, timeout_us);
    if (bytes_read < 1) return string("");
    text[bytes_read] = 0;
    return string((char*)text);
}

void UDPBroadcast::send(const uint8_t* data, size_t len) const
{
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(struct sockaddr_in));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_BROADCAST;   // ~0 means "broadcast"
    //inet_pton(AF_INET, "192.168.2.255", &(addr.sin_addr));
    addr.sin_port        = htons(_port);
    
    int bytes_sent = sendto(_s, data, len, 0L, (const sockaddr*)&addr, sizeof(addr));
    if (bytes_sent == -1)
    {
        char errmsg[256];
        char *err = strerror_r(errno, errmsg, sizeof(errmsg));
        ROS_ERROR("UDPBroadcast::send() failed: %s\n", err);
    }
    ROS_INFO("UDPBroadcast::send() success: len=%d\n", bytes_sent);
}


void UDPBroadcast::send(const string &text) const
{
    // Remove the carriage return, it makes things ugly
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(struct sockaddr_in));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_BROADCAST;   // ~0 means "broadcast"
    addr.sin_port        = htons(_port);
    
    char line[1024];
    gethostname(line, sizeof(line));

    string msg = string(line) + ":[" + std::to_string(getpid()) + "]:" + text;

    int bytes_sent = sendto(_s, msg.c_str(), msg.size(), 0L, 
                            (const sockaddr*)&addr, sizeof(addr));
    if (bytes_sent == -1)
    {
        char errmsg[256];
        char *err = strerror_r(errno, errmsg, sizeof(errmsg));
        ROS_ERROR("UDPBroadcast::send(string) failed: %s\n", err);
    }
}
