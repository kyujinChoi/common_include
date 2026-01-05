#ifndef UTIL_NETWORK_H_
#define UTIL_NETWORK_H_

#include <cstring>
#include <cstdio>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <netinet/if_ether.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <vector>
#include <unistd.h>
#include <fcntl.h>

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
inline int open_interface(const std::string &ifname)
{
    int sockfd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (sockfd < 0)
    {
        perror(("socket(AF_PACKET) failed on " + ifname).c_str());
        return -1;
    }
    int ifindex = if_nametoindex(ifname.c_str());
    if (ifindex == 0)
        return -1;
    sockaddr_ll sll{};
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = ifindex;
    sll.sll_protocol = htons(ETH_P_ALL);

    if (bind(sockfd, reinterpret_cast<sockaddr *>(&sll), sizeof(sll)) < 0)
    {
        perror(("bind(AF_PACKET) failed on " + ifname).c_str());
        close(sockfd);
        return -1;
    }
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags != -1)
    {
        fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
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
inline ssize_t read_from_udp(int sockfd, std::vector<uint8_t>& buf, int size)
{
    struct sockaddr_in sender_address;

    socklen_t sender_address_len = sizeof(sender_address);
    ssize_t nbytes = recvfrom(sockfd, buf.data(), size,
                                  0, (struct sockaddr *)&sender_address, &sender_address_len);
    return nbytes;
}
inline std::string mac_to_string(const uint8_t mac[ETH_ALEN])
{
    char buf[18]; // "xx:xx:xx:xx:xx:xx"
    std::snprintf(buf, sizeof(buf),
                  "%02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2],
                  mac[3], mac[4], mac[5]);
    return std::string(buf);
}
inline int open_interface(const std::string &ifname, int ifindex)
{
    int sockfd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (sockfd < 0)
    {
        perror(("socket(AF_PACKET) failed on " + ifname).c_str());
        return -1;
    }

    sockaddr_ll sll{};
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = ifindex;
    sll.sll_protocol = htons(ETH_P_ALL);

    if (bind(sockfd, reinterpret_cast<sockaddr *>(&sll), sizeof(sll)) < 0)
    {
        perror(("bind(AF_PACKET) failed on " + ifname).c_str());
        close(sockfd);
        return -1;
    }
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags != -1)
    {
        fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
    }
    return sockfd;
}
inline ether_header* get_ether_header(std::vector<uint8_t>& buf)
{
    auto *eth = reinterpret_cast<ether_header *>(buf.data());
    if (ntohs(eth->ether_type) != ETHERTYPE_IP)
        return nullptr; // IPv4만
    return eth;
}
inline iphdr* get_ip_header(std::vector<uint8_t>& buf)
{
    size_t offset = sizeof(ether_header);
    if (static_cast<size_t>(buf.size()) < offset + sizeof(iphdr))
        return nullptr;
    auto *ip = reinterpret_cast<iphdr *>(buf.data() + offset);
    if (ip->version != 4)
        return nullptr;
    size_t ip_header_len = ip->ihl * 4;
    if (static_cast<size_t>(buf.size()) < offset + ip_header_len + sizeof(udphdr))
        return nullptr;
    if (ip->protocol != IPPROTO_UDP)
        return nullptr;
    return ip;
}
inline udphdr* get_udp_header(std::vector<uint8_t>& buf)
{
    size_t offset = sizeof(ether_header);
    auto *ip = reinterpret_cast<iphdr *>(buf.data() + offset);
    size_t ip_header_len = ip->ihl * 4;
    auto *udp = reinterpret_cast<udphdr *>(buf.data() + offset + ip_header_len);
    uint16_t udp_len = ntohs(udp->len);
    if (udp_len < sizeof(udphdr))
        return nullptr;
    return udp;
}
inline std::string get_src_ip(iphdr* ip)
{
    in_addr src_addr{};
    src_addr.s_addr = ip->saddr;
    char ip_str[INET_ADDRSTRLEN];
    if (!inet_ntop(AF_INET, &src_addr, ip_str, sizeof(ip_str)))
    {
        return "";
    }
    return std::string(ip_str);
}
inline int get_src_port(udphdr* udp)
{
    return ntohs(udp->source);
}
inline std::string get_dst_ip(iphdr* ip)
{
    in_addr src_addr{};
    src_addr.s_addr = ip->daddr;
    char ip_str[INET_ADDRSTRLEN];
    if (!inet_ntop(AF_INET, &src_addr, ip_str, sizeof(ip_str)))
    {
        return "";
    }
    return std::string(ip_str);
}
inline int get_dst_port(udphdr* udp)
{
    return ntohs(udp->dest);
}
#endif