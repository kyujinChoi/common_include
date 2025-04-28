#ifndef UTIL_NETWORK_H_
#define UTIL_NETWORK_H_

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstring>
#include <stdio.h>
#define MAX_BUFFER_SIZE 99999
typedef unsigned char uint8;

inline int startup_tcp(std::string ip, int port)
{
    int sockfd = socket(PF_INET, SOCK_STREAM, 0);
    int option = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(int));
    if (sockfd == -1)
    {
        perror("socket error");
        return -1;
    }

    struct sockaddr_in my_addr;           // my address information
    memset(&my_addr, 0, sizeof(my_addr)); // initialize to zeros
    my_addr.sin_family = PF_INET;         // host byte order
    my_addr.sin_port = htons(port);       // port in network byte order
    my_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    int ret = connect(sockfd, (struct sockaddr *)&my_addr, sizeof(my_addr));
    if (ret == -1)
    {
        printf("Connection Failed");
        exit(-1);
    }
    
    return sockfd;
}
inline int sendMsg(int sockfd, std::string msg)
{
    int size = 0;
    if ((size = send(sockfd, &msg[0], msg.size(), 0)) < 0)
    {
        perror("send error");
    }
    return size;
}
inline std::string recvMsg(int sockfd)
{
    std::string buffer(MAX_BUFFER_SIZE, '\0');
    int size = 0;
    while ((size = recv(sockfd, &buffer[0], MAX_BUFFER_SIZE, 0)) > 0)
    {
        buffer.resize(size);
        return buffer;
    }
    if(size == 0)
        return "disconnected";
    return "recvMsg error";
}
inline int startup_udp(int port)
{
    int sockfd = -1;
    int option = 1;
    sockfd = socket(PF_INET, SOCK_DGRAM, 0);
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(int));
    if (sockfd == -1)
    {
        perror("socket error");
        return -1;
    }

    struct sockaddr_in my_addr;                   // my address information
    memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
    my_addr.sin_family = AF_INET;          // host byte order
    my_addr.sin_port = htons(port); // port in network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

    if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1)
    {
        perror("bind error");
        return -1;
    }

    printf("SERVER_PORTNO fd is %d\n", port);
    printf("lidar socket fd is %d\n", sockfd);
    return sockfd;
}
inline ssize_t read_from_udp(int sockfd, uint8* buf, int size)
{
    struct sockaddr_in sender_address;

    socklen_t sender_address_len = sizeof(sender_address);
    ssize_t nbytes = recvfrom(sockfd, buf, size,
                                  0, (struct sockaddr *)&sender_address, &sender_address_len);
    return nbytes;
}
#endif