#include "socket_comm/socketClient.h"
#include "socket_comm/socketServer.h"

int main(int argc, char **argv)
{
    int tmp=123;
    std::thread t([](const void* user)
    {
        SocketClient client("127.0.0.1", 1234, user);
        client.run([](SocketClient* client)
        { 
            std::cout << "in client : " << *(int*)client->m_pUser << std::endl;
            char rMessage[MAX_BUFFER] = "hello world";
            int size;
            if ((size = write(client->sockfd, rMessage, MAX_BUFFER)) < 0)
            {
                perror("write error");
            }
        });
        sleep(1);
        client.run([](SocketClient* client)
        { 
            char rMessage[MAX_BUFFER] = "hello world hello world";
            int size;
            if ((size = write(client->sockfd, rMessage, MAX_BUFFER)) < 0)
            {
                perror("write error");
            }
        });
    }, (const void*)&tmp);
    t.detach();
    SocketServer server(1234,(const void*)&tmp), multi_server;
    server.run_single([](Client * client, const void* user)
    { 
        char sMessage[MAX_BUFFER] = "send from server message";
        char rMessage[MAX_BUFFER];
        
        int size;
        
        std::cout << *(int*)user << std::endl;
        while ((size = recv(client->sockfd, rMessage, MAX_BUFFER, 0)) > 0)
        {
            rMessage[size] = '\0';
            printf("SERVER : %s\n", rMessage);        
        }
    });

    
    // multi_server.addHandler([](Client * client)
    // { 
    //     char sMessage[MAX_BUFFER] = "message1";
    //     char rMessage[MAX_BUFFER];
        
    //     int size;
    //     if ((size = write(client->sockfd, sMessage, MAX_BUFFER)) < 0)
    //     {
    //         perror("write error");
    //     }
    //     // printf("SERVER send: %s\n", sMessage);  

    //     while ((size = recv(client->sockfd, rMessage, MAX_BUFFER, 0)) > 0)
    //     {
    //         rMessage[size] = '\0';
    //         printf("SERVER receive: %s\n", rMessage);        
    //     }
    // });
    // multi_server.addHandler([](Client * client)
    // { 
    //     char sMessage[MAX_BUFFER] = "message2";
    //     char rMessage[MAX_BUFFER];
        
    //     int size;
    //     if ((size = write(client->sockfd, sMessage, MAX_BUFFER)) < 0)
    //     {
    //         perror("write error");
    //     }
    //     // printf("SERVER2 send: %s\n", sMessage);  

    //     while ((size = recv(client->sockfd, rMessage, MAX_BUFFER, 0)) > 0)
    //     {
    //         rMessage[size] = '\0';
    //         printf("SERVER2 receive: %s\n", rMessage);        
    //     }
    // });
    // std::thread t([]()
    // {
    //     SocketClient client("127.0.0.1", 1235);
    //     client.run([](SocketClient* client)
    //     { 
    //         char rMessage[MAX_BUFFER];
    //         int size;
    //         while((size=recv(client->sockfd, rMessage, MAX_BUFFER, 0))>0)
    //         {
    //             rMessage[size] = '\0';
    //             printf("CLIENT : %s\n", rMessage);        
    //         }
    //     });
    // });
    // t.detach();
    // std::thread t2([]()
    // {
    //     SocketClient client("127.0.0.1", 1235);
    //     client.run([](SocketClient* client)
    //     { 
    //         char rMessage[MAX_BUFFER];
    //         int size;
    //         while((size=recv(client->sockfd, rMessage, MAX_BUFFER, 0))>0)
    //         {
    //             rMessage[size] = '\0';
    //             printf("CLIENT2 : %s\n", rMessage);        
    //         }
    //     });
    // });
    // t2.detach();
    // multi_server.run_multiple(1235);
    
    return 0;
}