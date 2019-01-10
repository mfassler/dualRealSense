#ifndef __UDP_SENDER__H
#define __UDP_SENDER__H

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


class UdpSender {
private:
	const char *_ServerAddress;
	uint16_t _DataRxPort;
	uint16_t _ImageRxPort;
	struct sockaddr_in data_server;
	struct sockaddr_in image_server;
	int data_sockfd;
	int image_sockfd;

public:
	UdpSender(const char*, uint16_t, uint16_t);
	void sendData(unsigned char*, int);
	void sendImage(unsigned char*, int);
};




#endif // __UDP_SENDER__H
