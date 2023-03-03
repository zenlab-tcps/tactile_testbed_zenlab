#ifndef SocketCommHeader    /* This is an "include guard" */
#define SocketCommHeader    /* prevents the file from being included twice. */
                     /* Including a header file twice causes all kinds */
                     /* of interesting problems.*/

/* Headers */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <ctype.h>
#include <stdint.h>
#include <bits/types.h>

// Socket handling includes
#include <unistd.h>
#include <sys/types.h>
// Defines and creates socket
#include <sys/socket.h>
#include <arpa/inet.h>
//#include <stdint.h>
// Gives socket structure
#include <netinet/in.h>

#include <time.h>
#include <sys/time.h>

/* Helper Functions */
//  Return current system clock as microseconds
static int64_t s_clock (void)
{
#if (defined (WIN32))
    SYSTEMTIME st;
    GetSystemTime (&st);
    //return (int64_t) st.wSecond * 1000000 + st.wMilliseconds;
    return (int64_t) st.wSecond * 1000000 + st.wMicroseconds;
#else
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);
#endif
}

#define MAXLINE 256

//  Sleep for a number of milliseconds
static void s_sleep (int msecs)
{
#if (defined (WIN32))
    Sleep (msecs);
#else
    struct timespec t;
    t.tv_sec  =  msecs / 1000;
    t.tv_nsec = (msecs % 1000) * 1000;
    nanosleep (&t, NULL);
#endif
}
/* Helper Functions */


/**
 * This is a function declaration.
 * It tells the compiler that the function exists somewhere.
 */
int Get_UDP_SendSocket();
void FillDestinationPortInfo(const char *SendtoUDPSocket_IP, uint16_t portsend);
int Get_UDP_RecvSocket(const char *ReceiveUDPSocket_IP, uint16_t portrecv);
void closeSocket(int socket);
int send_payload(const void *message, size_t length);
int recv_payload(void *message);

#endif /* SocketCommHeader */
