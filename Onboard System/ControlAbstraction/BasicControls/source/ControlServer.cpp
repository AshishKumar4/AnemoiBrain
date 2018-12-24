#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>

#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread

#include "BasicControls.h"

/* ------------------------------------------------------------------------------------------------------------------------ */
/* --------------------------------------------------Some Configurations--------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

/*
  STREAM_PROTOCOL_1 Refers to conventional failproof streaming of values as packets, encapsulated and later stripped off on the other hand 
  STREAM_PROTOCOL_2 Refers to newer, faster and lightweight but simplest streaming, stream of simple bytes. This isn't failproof and errors 
                    may be entroduced.
*/
//#define STREAM_PROTOCOL_1 // FailProof, Encapsulate
#define STREAM_PROTOCOL_2 // Simple Byte Stream, Faster and lightweight

//#define DRONELESS_LOCAL_TEST

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------Permanent Configurations------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(STREAM_PROTOCOL_1)
#undef STREAM_PROTOCOL_2
#elif defined(STREAM_PROTOCOL_2)
#undef STREAM_PROTOCOL_1
#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------------------Main Program------------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

using namespace std;

//int server_fd, new_socket, valread;
//struct sockaddr_in address;
int opt = 1;

const char HANDSHAKE_IN_MSG[] = "Hello Gardien!";
const char HANDSHAKE_OUT_MSG[] = "Hello Overloard!";
typedef void (*func_t)(int); //void function pointer
func_t CHANNEL_HANDLER_TABLES[] = {&setThrottle, &setPitch, &setRoll, &setYaw, &setAux1, &setAux2};
string CHANNEL_NAME_TABLES[] = {"throttle", "pitch", "roll", "yaw", "aux1", "aux2"};

class ControlServer
{
    static vector<int> server_fd;
    static vector<int> socket_num;
    vector<thread *> ListenerThreads;
    static vector<struct sockaddr_in *> addresses;

    static int PORT_BASE;

  protected:
    int LaunchListenerThreads()
    {
    back:
        ListenerThreads.empty();
        ListenerThreads.push_back(new thread(ChannelListeners, 0));
        ListenerThreads.push_back(new thread(ChannelListeners, 1));
        ListenerThreads.push_back(new thread(ChannelListeners, 2));
        ListenerThreads.push_back(new thread(ChannelListeners, 3));
        ListenerThreads.push_back(new thread(ChannelListeners, 4));
        ListenerThreads.push_back(new thread(ChannelListeners, 5));

        ListenerThreads[0]->join();
        ListenerThreads[1]->join();
        ListenerThreads[2]->join();
        ListenerThreads[3]->join();
        ListenerThreads[4]->join();
        ListenerThreads[5]->join();
        //std::cout << "Broken Pipe, Waiting for incoming Connections...";
        return 0;
    }

  public:
    ControlServer(int portBase = 8400)
    {
        PORT_BASE = portBase;
        // We would first create several sockets and store their fds
        CreateChannel(portBase + 0, 0); //  Throttle is         0
        CreateChannel(portBase + 1, 1); //  Pitch is            1
        CreateChannel(portBase + 2, 2); //  Roll is             2
        CreateChannel(portBase + 3, 3); //  Yaw is              3
        CreateChannel(portBase + 4, 4); //  Aux1 is             4
        CreateChannel(portBase + 5, 5); //  Aux2 is             5
        // We Create six channels, each corresponding to six basic controls of drone

        // We would then create serveral threads, which would in turn listen to sockets
        LaunchListenerThreads();
    }
    ~ControlServer()
    {
        // TODO: Release all the sockets here
    }

    static int CreateChannel(int port, int channel); // This would create a port for a particular channel
    static void ChannelListeners(int i);
};

vector<int> ControlServer::server_fd;
vector<int> ControlServer::socket_num;
vector<struct sockaddr_in *> ControlServer::addresses;

int ControlServer::PORT_BASE;

void ControlServer::ChannelListeners(int i)
{
    int PORT = i + PORT_BASE;
    int sfd = server_fd[i];
    struct sockaddr_in *address = addresses[i];
    int new_socket;
    ssize_t valread = 0;
    char *buff = new char[4096];

    std::cout << "\n\nServer Initialized at port " << PORT << " Successfully...";
    if (listen(sfd, 5) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    std::cout << "\nwaiting for incoming Connections...";
    while (1)
    {
        int addrlen = sizeof(struct sockaddr);
        if ((new_socket = accept(sfd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
        {
            perror("accept");
            exit(EXIT_FAILURE);
        }
        std::cout << "\nGot an incoming request...\n";

        valread = read(new_socket, buff, 1024);
        if (valread == 0)
            continue;

        if (strncmp(buff, HANDSHAKE_IN_MSG, strlen(HANDSHAKE_IN_MSG)))
        {
            std::cout << "Overloard Could not establish Connection / Handshake Failure...\n";
            continue;
        }
        else
        {
            send(new_socket, HANDSHAKE_OUT_MSG, strlen(HANDSHAKE_OUT_MSG), 0);
            std::cout << "Overloard Connected Successfully...\n";
        }

        // The Control Loop -->
        while (1)
        {
            try
            {
                memset(buff, 0, 4096);
                valread = read(new_socket, buff, 4096);
                if (valread == 0 || valread == -1)
                    break;
#if defined(STREAM_PROTOCOL_1)
                std::string parsed, cmd(buff);
                //std::cout << cmd << endl;

                /*
                    Format of the input command ->
                    .[:x:].
                */
                std::stringstream input_stringstream(cmd);
                // Split the input line into several [:x:]
                while (std::getline(input_stringstream, parsed, '.'))
                {
                    if (!parsed.length())
                        continue;
                    //std::cout << "Parsed: [" << parsed << "],\n";
                    std::string par1, par2, val;
                    std::stringstream parsed_stream(parsed);
                    std::getline(parsed_stream, par1, ':');
                    std::getline(parsed_stream, val, ':'); // Extract the x

                    //std::cout << endl << par1 << " " << "<" << val.length() << ">" << val << " (" << atoi(val.c_str()) << ") " << par2 << "\n";

                    // We Now issue our command
                    //printf("\n [%s] Command Issued ", CHANNEL_NAME_TABLES[i].c_str());

#ifndef DRONELESS_LOCAL_TEST
                    CHANNEL_HANDLER_TABLES[i](atoi(val.c_str()));
#endif
                }
#elif defined(STREAM_PROTOCOL_2)
                for(int j = 0; j < valread; j++)
                {
                    CHANNEL_HANDLER_TABLES[i](int(buff[j]));
                }
#endif

                //send(new_socket, resbuff, sizeof(ResponsePackets), 0);
                //send(new_socket, resbuff, sizeof(ResponsePackets), 0);
                //printf("[message sent]\n");
            }
            catch (exception &e)
            {
                cout << "Some ERROR!!!" << e.what() << "\n";
            }
        }
        std::cout << "Broken Pipe, Waiting for incoming Connections...";
    }
}

int ControlServer::CreateChannel(int port, int channel) // This would create a port for a particular channel
{
    int sfd;
    struct sockaddr_in *address = new struct sockaddr_in;
    // Creating socket file descriptor
    if ((sfd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port
    if (setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address->sin_family = AF_INET;
    address->sin_addr.s_addr = INADDR_ANY;
    address->sin_port = htons(port);

    // Forcefully attaching socket to the given port
    bind(sfd, (struct sockaddr *)address, sizeof(struct sockaddr_in));
    /*if (bind(sfd, (struct sockaddr *)address, sizeof(struct sockaddr_in)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }*/
    try
    {
        if (server_fd.size() <= channel)
        {
            server_fd.push_back(sfd);
            addresses.push_back(address);
        }
        else
        {
            // TODO: Free up the existing sockets
            server_fd[channel] = sfd;
            addresses[channel] = address;
        }
    }
    catch (...)
    {
        perror("Error while Saving the Socket Memory Structures");
        exit(EXIT_FAILURE);
    }
    cout << "Socket for port " << port << " Created Successfully!" << endl;
    return 0;
}
int main(int argc, char const *argv[])
{
    char *buff = new char[1024];
#ifndef DRONELESS_LOCAL_TEST
    BasicControls_init(argc, (char **)argv); // Maybe lower levels can make use of command line args
#endif
    //resbuff = getResponse();
    //std::cout << "\nSPI Threads Initialized...\n";

    //Server_start();
    ControlServer *server = new ControlServer;
    while (1)
        ;
    return 0;
}
