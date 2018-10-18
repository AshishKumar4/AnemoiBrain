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

#include "BasicControls.cpp"

using namespace std;

#define PORT 8194

int server_fd, new_socket, valread;
struct sockaddr_in address;
int opt = 1;
int addrlen = sizeof(address);

int Server_start()
{

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *)&address,
             sizeof(address)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    return 0;
}

int main(int argc, char const *argv[])
{
    char *buff = new char[1024];
    BasicControls_init();
    std::cout<<"\nSPI Threads Initialized...\n";

    Server_start();
    std::cout << "\n\nServer Initialized at port " << PORT << " Successfully...";
    if (listen(server_fd, 5) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    std::cout << "\nwaiting for incoming Connections...";
    while (1)
    {
        std::cout << "\nGot an incoming request...\n";
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
        {
            perror("accept");
            exit(EXIT_FAILURE);
        }

        while (1)
        {
            valread = read(new_socket, buff, 1024);
            if (valread == 0)
                break;

            std::string parsed, cmd(buff);
            std::cout << cmd << endl;
            std::stringstream input_stringstream(cmd);

            while (std::getline(input_stringstream, parsed, ' '))
            {
		cout<<"["<<parsed<<"]";
                string par, val;
                std::stringstream parsed_stream(parsed);
                std::getline(parsed_stream, par, ':');
		std::getline(parsed_stream, val, ':');
		cout<<par<<" "<<val<<" ("<<stoi(val)<<")"<<endl;
                switch (par[0])
                {
                case 'T':
                    //std::cout << stoi(val);
                    setThrottle(stoi(val));
                    break;
                case 'R':
                    //std::cout << stoi(;
                    setRoll(stoi(val));
                    break;
                case 'Y':
                    //std::cout << val;
                    setYaw(stoi(val));
                    break;
                case 'P':
                    //std::cout << val;
                    setPitch(stoi(val));
                    break;
		case 'C':
			cout<<"\nDone...\n";
		    break;
                default:
                    std::cout << "Not Recognized!";
                }
            }

            send(new_socket, "Hello Client!!!", strlen("Hello Client!!!"), 0);
            printf("[message sent]\n");
        }
        cout << "Broken Pipe, Waiting for incoming Connections...";
    }
    return 0;
}
