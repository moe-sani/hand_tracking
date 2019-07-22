#ifndef _UDP_HPP_
#define _UDP_HPP_

/**
 * INCLUDES
 */
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>


/**
 * DEFINES
 */

// States
#define UDP_STATE_NOINIT		0
#define UDP_STATE_DISCONNECTED	1
#define UDP_STATE_CONNECTED		2

// Errors
#define UDP_ERROR_NOERROR		0
#define UDP_ERROR_GENERIC		1
#define UDP_ERROR_SEND			2
#define UDP_ERROR_RECEIVE		3
#define UDP_ERROR_OVERFLOW		4


//-----------
// UDP CLASS
//-----------

class Udp
{
private:
	// State and error
	int iState;
	int iError;

	// UDP connection
	int iUdp_socket;
	struct sockaddr_in stUdp_addr;


	// Pthreads
	pthread_mutex_t mUdp_mutex;
	pthread_t thUdp_thread;
	
	// Read buffer
	char *cBuffer_data;
	int iBuffer_length;
	
	// New data
	bool bData_received;

public:

    // New direction
    bool bDirn_received;
    int iDirection;

	/* Default Constructor */
	Udp();

	/* Default Destructor */
	~Udp();

	/** Establish UDP socket connection with address and port
	 *@return       true/false for success/failure
	 */
	bool connect(const char *cAddress, int iPort);

	/** Disconnect UDP socket
	 *@return       true/false for success/failure
	 */
	bool disconnect();
	
	/**Set size of UDP buffer for data reception
	 *@return       set buffer length
	 */
	int setBufferSize(int iSize);

	/**Send buffer of pre-determined length over socket
	 *@return       true/false for success/failure
	 */
	bool send(char *cData);

	/**Send buffer of 'size' length over socket
	 *@return       true/false for success/failure
	 */
	bool send(char *cData, int iSize);

	/**Receive buffer of pre-determined length over socket
	 *@return       true/false for success/failure
	 */
	bool receive(char *cData);

	/**Get status of UDP socket connection
	 *@return       state
	 */
	int getState();

	/**Get connection error status
	 *@return       error
	 */
	int getError();

private:

	/**run UDP thread to receive streaming data over socket
	 *
	 */
	static void *udp_thread(void *arg);
};

#endif
