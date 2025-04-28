#ifndef _NETWORK_CLIENT_H_
#define _NETWORK_CLIENT_H_
// #pragma once
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>
#include <vector>
#include <thread>
#include <functional>
#define MAX_BUFFER 9999

class SocketClient
{
public:
    SocketClient(std::string ip, int port, const void* pUser)
    {
        struct sockaddr_in server_addr;
        int val = 1;
        sockfd = socket(PF_INET, SOCK_STREAM, 0);
        // set a socket option to reuse the server address
        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)) != 0)
        {
            printf("error: setsockopt(): %s\n", strerror(errno));
            return;
        }
        server_addr.sin_family = PF_INET;
        server_addr.sin_port = htons(port);
        inet_aton(ip.c_str(), &server_addr.sin_addr);
        memset(&(server_addr.sin_zero), 0, 8);

        if(connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) ==-1)
        {
            std::cout << "[SocketClient] connect error\n";
            return ;
        }
        std::cout << "[SocketClient] connected!\n";
        m_pUser = pUser;
    };
    ~SocketClient(){};
    
    void run(std::function<void(SocketClient *)> func)
    {
        if(sockfd > 0)
            func(this);
        return;
    }
    void stop()
    {
        close(sockfd);
        return;
    }
    int sendMsg(std::string cmd)
    {
        int size = 0;
        if ((size = write(sockfd, &cmd[0], cmd.size())) < 0)
        {
            perror("write error");
        }
        return size;
    }
    std::string recvMsg()
    {
        std::string buffer(MAX_BUFFER, '\0');
        int size = 0;
        while ((size = recv(sockfd, &buffer[0], MAX_BUFFER, 0)) > 0)
        {
            buffer.resize(size);
            return buffer;
        }
        return "";
    }
public:
    int sockfd = -1;
    const void* m_pUser = 0;
};
#endif