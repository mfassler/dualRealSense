#ifndef __UDP_SENDER__H
#define __UDP_SENDER__H

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


class UdpSender {
private:
	const char *_ServerAddress;
	uint16_t _ServerRxPort;
	struct sockaddr_in data_server;
	int data_sockfd;

public:
	UdpSender(const char*, uint16_t);
	void sendData(unsigned char*, int);
	void sendImage(unsigned char*, int);
};




#endif // __UDP_SENDER__H
