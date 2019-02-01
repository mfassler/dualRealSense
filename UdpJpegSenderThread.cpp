
#include <netinet/in.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


#define START_MAGIC "__HylPnaJY_START_JPG %09d\n"
#define STOP_MAGIC "_g1nC_EOF"
#define STOP_MAGIC_LEN 9


void network_jpg_sender_thread(int udp_sockfd, cv::Mat out_image, struct sockaddr_in jpgRxAddr) {
	std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 30};
	std::vector<uchar> outputBuffer;

	if (out_image.cols > 10 && out_image.rows > 10) {
		if (!cv::imencode("*.jpg", out_image, outputBuffer, params)) {
			printf("failed to imencode\n");
		} else {
			unsigned char* outBuffer = (unsigned char*)&outputBuffer[0];
			int bufLen = outputBuffer.size();

			int filepos = 0;
			int numbytes = 0;

			char startString[32];
			int startStringLen;

			startStringLen = sprintf(startString, START_MAGIC, bufLen);

			// Send the jpg image out via UDP:
			sendto(udp_sockfd, startString, startStringLen, 0,
			   (const struct sockaddr*)&jpgRxAddr, sizeof(jpgRxAddr));
			while (filepos < bufLen) {
				if (bufLen - filepos < 1400) {
					numbytes = bufLen - filepos;
				} else {
					numbytes = 1400;  // Ethernet MTU is 1500
					// (... but the max *safe* UDP payload is 508 bytes...)
				}

				sendto(udp_sockfd, &outBuffer[filepos], numbytes, 0,
					   (const struct sockaddr*)&jpgRxAddr, sizeof(jpgRxAddr));

				filepos += numbytes;
			}

			sendto(udp_sockfd, STOP_MAGIC, STOP_MAGIC_LEN, 0,
				   (const struct sockaddr*)&jpgRxAddr, sizeof(jpgRxAddr));

		}
	}
}



