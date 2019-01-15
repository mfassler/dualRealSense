
#include <stdio.h>
#include <strings.h>  // for bzero()

#include "UdpSender.hpp"



UdpSender::UdpSender(const char *ServerAddress, uint16_t ServerRxPort) {

	_ServerAddress = ServerAddress;
	_ServerRxPort = ServerRxPort;

	bzero((char*)&data_server, sizeof(data_server));
	data_server.sin_family = AF_INET;
	data_server.sin_addr.s_addr = inet_addr(_ServerAddress);
	data_server.sin_port = htons(_ServerRxPort);

	data_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (data_sockfd < 0) {
		fprintf(stderr, "Error opening socket");
		//return EXIT_FAILURE;
	}
}



void UdpSender::sendData(unsigned char *outBuffer, int bufLen) {
	sendto(data_sockfd, outBuffer, bufLen, 0,
	       (const struct sockaddr*)&data_server, sizeof(data_server));
}


void UdpSender::sendImage(unsigned char *outBuffer, int bufLen) {
	#define START_MAGIC "__HylPnaJY_START_JPG %09d\n"
	#define STOP_MAGIC "_g1nC_EOF"
	#define STOP_MAGIC_LEN 9

	int filepos = 0;
	int numbytes = 0;

	char startString[32];
	int startStringLen;

	startStringLen = sprintf(startString, START_MAGIC, bufLen);

	// Send the jpg image out via UDP:
	sendto(data_sockfd, startString, startStringLen, 0,
	       (const struct sockaddr*)&data_server, sizeof(data_server));

	while (filepos < bufLen) {
		if (bufLen - filepos < 1400) {
			numbytes = bufLen - filepos;
		} else {
			numbytes = 1400;  // Ethernet MTU is 1500
			// (... but the max *safe* UDP payload is 508 bytes...)
		}

		sendto(data_sockfd, &outBuffer[filepos], numbytes, 0,
		       (const struct sockaddr*)&data_server, sizeof(data_server));

		filepos += numbytes;
	}

	sendto(data_sockfd, STOP_MAGIC, STOP_MAGIC_LEN, 0,
	       (const struct sockaddr*)&data_server, sizeof(data_server));
}



