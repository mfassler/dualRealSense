#ifndef __UDP_JPG_SENDER_THREAD_
#define __UDP_JPG_SENDER_THREAD_


#include <netinet/in.h>
#include <opencv2/core.hpp>


extern void network_jpg_sender_thread(int, cv::Mat, struct sockaddr_in);


#endif // __UDP_JPG_SENDER_THREAD_
