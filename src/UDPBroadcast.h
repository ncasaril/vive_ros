#ifndef UDPBROADCAST_H
#define UDPBROADCAST_H

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <netdb.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <string>

using namespace std;

class UDPBroadcast
{
public:
    UDPBroadcast(int port);
    ~UDPBroadcast() {close(_s);};

    // My own return codes, to make the code's intent a bit more obvious
    enum {
        B_ERROR = -1,
        B_NO_ERROR
    };

    // My return type
    typedef int status_t;

    string listen(int timeout_us = -1) const;
    void send(const string &text) const;

    size_t listen(uint8_t* buffer, size_t buffer_size, int timeout_us = -1) const;
    void send(const uint8_t* data, size_t len) const;

    static void send(int port, const string &text)
    {
        UDPBroadcast u(port);
        u.send(text);
    }

private:
    // Enables or disables blocking I/O mode on the given socket
    status_t SetSocketBlockingEnabled(int sock, bool blocking)
    {
        int flags = fcntl(sock, F_GETFL, 0);
        if (flags < 0) return B_ERROR;
        flags = blocking ? (flags&~O_NONBLOCK) : (flags|O_NONBLOCK);
        return (fcntl(sock, F_SETFL, flags) == 0) ? B_NO_ERROR : B_ERROR;
    }
    
    // Binds this socket to the broadcast address, so that it will receive broadcast UDP packets on (port)
    status_t BindUDPSocketToBroadcast(int sock, int port)
    {
        // This allows multiple processes will be allowed to bind to the port at once.
        int one = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char *) &one, sizeof(one));
        
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(struct sockaddr_in));
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;   // zero means "accept data from any IP address"
        addr.sin_port        = htons(port);
        int ret = ::bind(sock, (struct sockaddr *) &addr, sizeof(addr));
        if (ret == 0) return B_NO_ERROR;
        else return B_ERROR;
    }
    
    // Enables/disables UDP broadcast send/receive on the given socket.
    status_t SetUDPSocketBroadcastEnabled(int sock, bool enableBroadcast)
    {
        int val = (enableBroadcast ? 1 : 0);
        return (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char *) &val, sizeof(val)) == 0) ? B_NO_ERROR : B_ERROR;
    }
    
    // Tells the socket that whenever we call send() on it, it should send the data to (broadcast) on (port)
    status_t SetUDPSocketTargetToBroadcast(int sock, int port)
    {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(struct sockaddr_in));
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_BROADCAST;   // ~0 means "broadcast"
        addr.sin_port        = htons(port);
        
        return (connect(sock, (struct sockaddr *) &addr, sizeof(addr)) == 0) ? B_NO_ERROR : B_ERROR;
    }
    
    int _s;
    int _port;
};


#endif
