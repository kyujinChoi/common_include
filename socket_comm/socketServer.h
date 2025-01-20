#ifndef _NETWORK_SERVER_H_
#define _NETWORK_SERVER_H_
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

struct Client
{
    int id = -1;
    int sockfd = -1;
    struct sockaddr_in sockaddr;
};

class SocketServer
{
public:
    SocketServer(){}; // for multi socket
    SocketServer(int port, const void *pUser) // for single socket
    {
        int halfsd = init(port);
        if(halfsd < 0) 
            return ;
        struct sockaddr_in client_addr;
        int len = sizeof(client_addr);
        int fullsd = accept(halfsd, (struct sockaddr *)&client_addr, (socklen_t *)&len);
        if (fullsd == -1)
        {
            perror("accept error");
            return;
        }
        std::cout << "[SocketServer] Connected\n";
        m_pUser = pUser;
        Client *client = new Client();
        client->id = id;
        client->sockfd = fullsd;
        client->sockaddr = client_addr;
        clients.resize(1);
        clients[0] = client;
        return;
    };
    ~SocketServer(){};

    void addHandler(std::function<int(Client *, const void *)> func, const void *pUser)
    {
        Client *client = new Client();
        
        client->id = id;
        threads.resize(id + 1);
        threads[id] = std::thread([](std::function<int(Client *, const void *)> func, Client* client, const void *pUser) 
        {
            while(1)
            {
                usleep(1000);

                if(client->sockfd > 0)
                {
                    int result = func(client, pUser);
                    if (result == 0)
                    {
                        close(client->sockfd);
                        client->sockfd = -1;
                        continue;
                    }
                }
            }
            
        }, func, client, pUser);
        // threads.push_back(t);
        threads[id].detach();
        clients.push_back(client);
        ++id;
        
        return;
    }
    int run_multiple(int port) // call addHandler before call run_multiple
    {
        int halfsd = init(port);
        if(halfsd < 0) 
            return -1;
        struct sockaddr_in client_addr;
        int len = sizeof(client_addr);
        while(1)
        {
            int fullsd = accept(halfsd, (struct sockaddr *)&client_addr, (socklen_t *)&len);
            if (fullsd == -1)
            {
                perror("accept");
                return 0;
            }
            std::cout << "[SocketServer] Connected\n";
            int idx = findEmptyClient();
            clients[idx]->sockfd = fullsd;
            clients[idx]->sockaddr = client_addr;
        }
    }
    void run_single(std::function<int(Client *, const void *)> func)
    {
        func(clients[0], m_pUser);
        
        return;
    }
    void stop_single()
    {
        deleteClient(0);
        return;
    }
    int sendMsg(int sockfd, std::string cmd)
    {
        int size = 0;
        if ((size = write(sockfd, &cmd[0], cmd.size())) < 0)
        {
            perror("write error");
        }
        return size;
    }
    std::string recvMsg(int sockfd)
    {
        std::string buffer(MAX_BUFFER, '\0');
        int size = 0;
        while ((size = recv(sockfd, &buffer[0], MAX_BUFFER, 0)) > 0)
        {
            buffer.resize(size);
            return buffer;
        }
        if(size == 0)
            return "disconnected";
        return "error";
    }
private:
    int init(int port)
    {
        struct sockaddr_in sockaddr;
        int val = 1;

        // create an unnamed socket
        int sockfd = socket(AF_INET, SOCK_STREAM, 0);

        // set a socket option to reuse the server address
        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)) != 0)
        {
            printf("error: setsockopt(): %s\n", strerror(errno));
            return -1;
        }

        // name the socket with the server address
        sockaddr.sin_family = AF_INET;
        // sockaddr.sin_addr.s_addr = inet_addr(ipaddr);
        sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
        sockaddr.sin_port = htons(port);
        memset(&sockaddr.sin_zero, 0, 8);

        if (bind(sockfd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
        {
            printf("error: bind(): %s\n", strerror(errno));
            return -1;
        }

        // set the maximum number of pending connection requests
        if (listen(sockfd, 10) != 0)
        {
            printf("error: listen(): %s\n", strerror(errno));
            return -1;
        }

        return sockfd;
    }

    int findEmptyClient()
    {
        int idx = -1;
        for(int i = 0; i < clients.size(); ++i)
        {
            if(clients[i]->sockfd == -1)
            {
                idx = i;
                break;
            }
        }
        return idx;
    }
    void deleteClient(int idx)
    {
        if(idx >= clients.size()) return;
        close(clients[idx]->sockfd);
        clients[idx]->id = -1;
        clients[idx]->sockfd = -1;
        // clients.erase(clients.begin() + idx);
        return;
    }
public:
    std::vector<Client*> clients;
    std::vector<std::thread> threads;
    int id = 0;
    const void* m_pUser = 0;
};
#endif