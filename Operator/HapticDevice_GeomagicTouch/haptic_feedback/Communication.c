/*
This file contains all functions and definitions related
to communication
*/

// Include the header file for Socket Communication
#include "Communication.h"

static int socksendfd, sockrecvfd;
static socklen_t address_length_send, address_length_recv;
static struct sockaddr_in UDPsendservaddr, UDPrecvcliaddr;

int Get_UDP_SendSocket()
{
	// Creating UDP socket file descriptor for sending
	if ( (socksendfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}

    return socksendfd;
}

void FillDestinationPortInfo(const char *SendtoUDPSocket_IP, uint16_t portsend)
{
	memset(&UDPsendservaddr, 0, sizeof(UDPsendservaddr));

	// Filling UDP socket server information to send to
	UDPsendservaddr.sin_family = AF_INET; // IPv4
	UDPsendservaddr.sin_addr.s_addr = inet_addr(SendtoUDPSocket_IP);// INADDR_ANY; // Any address because this socket will just be used to send data
	UDPsendservaddr.sin_port = htons(portsend);

    address_length_send = sizeof(UDPsendservaddr);
}

int Get_UDP_RecvSocket(const char *ReceiveUDPSocket_IP, uint16_t portrecv)
{

    memset(&UDPrecvcliaddr, 0, sizeof(UDPrecvcliaddr));

    // Filling UDP socket server information to receive from
	UDPrecvcliaddr.sin_family = AF_INET; // IPv4
	UDPrecvcliaddr.sin_addr.s_addr = inet_addr(ReceiveUDPSocket_IP);
	UDPrecvcliaddr.sin_port = htons(portrecv);

    address_length_recv = sizeof(UDPrecvcliaddr);

	// Creating UDP socket for receiving
	if ( (sockrecvfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
    }
	struct timeval tv;
	tv.tv_sec = 0; 
	tv.tv_usec = 500; // 0.5ms config 
	// tv.tv_sec = 1;
	// tv.tv_usec = 0;  // 1s config
	if (setsockopt(sockrecvfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
		perror("Error");
	}

	// Bind the socket with the server recv address
	if ( bind(sockrecvfd, (const struct sockaddr *)&UDPrecvcliaddr,
	                                sizeof(UDPrecvcliaddr)) < 0 )
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}

	return sockrecvfd;
}

void closeSocket(int socket)
{
    close(socket);
}

int send_payload(const void *message, size_t length)
{
    // Sending message from arbitrary port using created socket
    int sentbytes = sendto(socksendfd, (const char *)message, length, 0,
        (const struct sockaddr *)&UDPsendservaddr,
            address_length_send);
    return sentbytes;
}

int recv_payload(void *message)
{
    int recvbytes = recvfrom(sockrecvfd, (char *)message, MAXLINE,
            MSG_WAITALL, ( struct sockaddr *) &UDPrecvcliaddr,
            &address_length_recv);
    return recvbytes;
}
