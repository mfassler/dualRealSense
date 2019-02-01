/*
 * dualRealSense
 *
 * Use two Intel RealSenses, mounted side-by-side
 *
 * Copyright 2019 Mark Fassler
 * Licensed under the GPLv3
 *
 */


#include <stdio.h>
#include <sys/time.h>  // gettimeofday
#include <sys/stat.h>

#include <thread>

#include "librealsense2/rs.hpp"
#include "librealsense2/rsutil.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "UdpJpegSenderThread.hpp"

cv::Mat color_image[2];
cv::Mat depth_image[2];
cv::Mat out_image;



double gettimeofday_as_double() {
	struct timeval tv;
	gettimeofday(&tv, NULL);

	double ts = tv.tv_sec;
	ts += (tv.tv_usec / 1000000.0);

	return ts;
}



unsigned char vid_frame_buffer[2][848*480*3];
unsigned char dep_frame_buffer[2][848*480*2];

void get_frame_thread(rs2::frameset frames, int cam_number) {

	rs2::align align(RS2_STREAM_COLOR);
	auto processed = align.process(frames);
	rs2::video_frame vid_frame = processed.get_color_frame();
	rs2::depth_frame dep_frame = processed.get_depth_frame();

	// TODO, FIXME:  when this thread ends, sometimes dep_frame is destroyed
	// before main() has a chance to use the data, causing a sigsegv.  
	// TODO:  how to keep the actual data in scope without doing a copy...
	memcpy(vid_frame_buffer[cam_number], vid_frame.get_data(), 848*480*3);
	memcpy(dep_frame_buffer[cam_number], dep_frame.get_data(), 848*480*2);

	color_image[cam_number] = cv::Mat(cv::Size(vid_frame.get_width(), vid_frame.get_height()),
		CV_8UC3, vid_frame_buffer[cam_number], cv::Mat::AUTO_STEP);

	depth_image[cam_number] = cv::Mat(cv::Size(dep_frame.get_width(), dep_frame.get_height()),
		CV_16UC1, dep_frame_buffer[cam_number], cv::Mat::AUTO_STEP);

	for (int i=0; i<depth_image[cam_number].rows; ++i) {
		for (int j=0; j<depth_image[cam_number].cols; ++j) {
			// A little counter-intuitive, but:
			// The depth image has "shadows".  The Intel librealsense2 driver interprets
			// shadows as distance == 0.  But we will change that to distance=max, so that
			// everything else will ignore the shadows:
			if (depth_image[cam_number].at<uint16_t>(i, j) < 20) {
				depth_image[cam_number].at<uint16_t>(i, j) = 65535;
			}
		}
	}
}





int main(int argc, char* argv[]) {

	// ##########################################################
	// ##########################################################
	//     BEGIN:  Read in config file
	// ##########################################################
	// ##########################################################
	std::string filename = "config.yaml";
	if (argc == 2) {
		filename = argv[1];
	}
	if (argc > 2) {
		printf("Usage: %s [optional_config_file.yaml]\n", argv[0]);
		exit(1);
	}

	// Check if the file exists:
	struct stat _sbuffer;
	if (stat(filename.c_str(), &_sbuffer) != 0) {
		printf("Can't access config file: %s\n", filename.c_str());
		exit(1);
	}

	printf("Using config file: %s\n", filename.c_str());

	cv::FileStorage fs;
	fs.open(filename, cv::FileStorage::READ);

	cv::FileNode fn_lcs = fs["left_camera_serial"];
	cv::FileNode fn_rcs = fs["right_camera_serial"];

	if (fn_lcs.type() == cv::FileNode::NONE) {
		printf("Config file is missing:  left_camera_serial\n");
		exit(1);
	}
	if (fn_rcs.type() == cv::FileNode::NONE) {
		printf("Config file is missing:  right_camera_serial\n");
		exit(1);
	}
	if (fn_lcs.type() != cv::FileNode::STR) {
		printf("Config file:  left_camera_serial should be a string\n");
		exit(1);
	}
	if (fn_rcs.type() != cv::FileNode::STR) {
		printf("Config file:  right_camera_serial should be a string\n");
		exit(1);
	}

	std::string left_camera_serial = fn_lcs.string();
	std::string right_camera_serial = fn_rcs.string();

	fs.release();

	printf("left_camera_serial: %s\n", left_camera_serial.c_str());
	printf("right_camera_serial: %s\n", right_camera_serial.c_str());

	// ##########################################################
	// ##########################################################
	//     END:  Read in config file
	// ##########################################################
	// ##########################################################

	// TODO:  move these into the config file:
	struct sockaddr_in jpgRxAddr;
	struct sockaddr_in scanline_rx_addr[1];
	int number_of_scanline_rxs = 1;
	jpgRxAddr.sin_family = AF_INET;
	jpgRxAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	jpgRxAddr.sin_port = htons(3123);

	scanline_rx_addr[0].sin_family = AF_INET;
	scanline_rx_addr[0].sin_addr.s_addr = inet_addr("127.0.0.1");
	scanline_rx_addr[0].sin_port = htons(3125);



	int udp_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (udp_sockfd < 0) {
		perror("failed to open UDP socket");
		exit(1);
	}

	unsigned char scanOut[ 848 * 2 * 2 ];  // will be larger than ethernet MTU  :-( ...

	rs2::pipeline pipeline_0;
	rs2::pipeline pipeline_1;

	rs2::config config_0;
	rs2::config config_1;

	config_0.enable_device(left_camera_serial);
	config_0.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 30);
	config_0.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
	//config_0.enable_device_from_file(argv[1], false);


	config_1.enable_device(right_camera_serial);
	config_1.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 30);
	config_1.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
	//config_1.enable_device_from_file(argv[2], false);

	printf("Attempting to start the pipelines...\n");
	rs2::pipeline_profile profile_0;
	rs2::pipeline_profile profile_1;
	try {
		profile_0 = pipeline_0.start(config_0);
		profile_1 = pipeline_1.start(config_1);
	} catch (const rs2::error& e) {
		printf("...pipelines failed.\n");
		printf("\n");
		printf("  - perhaps wrong serial number in config file?\n");
		printf("  - perhaps the cameras aren't being seen as USB-3 devices?\n");
		printf("       (RealSenses don't work too well as USB-2 devices)\n");
		printf("\n");
		exit(1);
	}
	printf("...started.\n");

	auto rgb_stream_0 = profile_0.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	auto rgb_stream_1 = profile_1.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

	const rs2_intrinsics rgb_intrinsics_0 = rgb_stream_0.get_intrinsics();
	const rs2_intrinsics rgb_intrinsics_1 = rgb_stream_1.get_intrinsics();

	// Left camera is upside-down:
	rs2_intrinsics rgb_intrinsics_0a;
	rgb_intrinsics_0a.width     = rgb_intrinsics_0.width;
	rgb_intrinsics_0a.height    = rgb_intrinsics_0.height;
	rgb_intrinsics_0a.ppx       = rgb_intrinsics_0.width - rgb_intrinsics_0.ppx;
	rgb_intrinsics_0a.ppy       = rgb_intrinsics_0.height - rgb_intrinsics_0.ppy;
	rgb_intrinsics_0a.fx        = rgb_intrinsics_0.fx;
	rgb_intrinsics_0a.fy        = rgb_intrinsics_0.fy;
	rgb_intrinsics_0a.model     = rgb_intrinsics_0.model;
	rgb_intrinsics_0a.coeffs[0] = rgb_intrinsics_0.coeffs[0];
	rgb_intrinsics_0a.coeffs[1] = rgb_intrinsics_0.coeffs[1];
	rgb_intrinsics_0a.coeffs[2] = rgb_intrinsics_0.coeffs[2];
	rgb_intrinsics_0a.coeffs[3] = rgb_intrinsics_0.coeffs[3];
	rgb_intrinsics_0a.coeffs[4] = rgb_intrinsics_0.coeffs[4];

	printf("orig: %d %d  %f %f %f %f\n", rgb_intrinsics_0.width, rgb_intrinsics_0.height, rgb_intrinsics_0.ppx, rgb_intrinsics_0.ppy, rgb_intrinsics_0.fx, rgb_intrinsics_0.fy);

	// -----------
	//  BEGIN:  only used for playback of .bag files
	// ----------
	//auto playback_0 = profile_0.get_device().as<rs2::playback>();
	//auto playback_1 = profile_1.get_device().as<rs2::playback>();
	//playback_0.seek( (std::chrono::nanoseconds) 55000000000);
	//playback_1.seek( (std::chrono::nanoseconds) 55115000000);
	//playback_1.seek( (std::chrono::nanoseconds) 115000000);
	// -----------
	//  END:  only used for playback of .bag files
	// ----------



	auto depth_sensor_0 = profile_0.get_device().first<rs2::depth_sensor>();
	float depth_scale = depth_sensor_0.get_depth_scale();
	printf("Depth scale: %.6f\n", depth_scale);


	double fps = 0.0;
	double ts0 = gettimeofday_as_double();
	double ts1;
	double tFrame;

	while (true) {

#ifdef USE_NETWORK_DISPLAY
		std::thread t2(network_jpg_sender_thread, udp_sockfd, out_image, jpgRxAddr);
#endif
		rs2::frameset frames_0 = pipeline_0.wait_for_frames();
		rs2::frameset frames_1 = pipeline_1.wait_for_frames();

		std::thread t0(get_frame_thread, frames_0, 0);
		std::thread t1(get_frame_thread, frames_1, 1);


		t0.join();
		t1.join();


		if (color_image[0].rows > 10 && color_image[0].rows == color_image[1].rows) {

			// Left camera is upside-down:
			cv::Mat color_image_0a;
			cv::Mat depth_image_0a;
			cv::flip(color_image[0], color_image_0a, -1);
			cv::flip(depth_image[0], depth_image_0a, -1);

#ifdef USE_NETWORK_DISPLAY
			t2.join();
#endif
			cv::hconcat(color_image_0a, color_image[1], out_image);

			// http://longstryder.com/2014/07/which-way-of-accessing-pixels-in-opencv-is-the-fastest/
			// Get a scan of the very middle row (the horizon, in theory)
			const unsigned char *ptr;
			ptr = depth_image_0a.ptr(240);
			for (int col=0; col < 848*2; ++col) {
				scanOut[col] = ptr[col];
			}
			ptr = depth_image[1].ptr(240);
			for (int col=0; col < 848*2; ++col) {
				scanOut[col+848*2] = ptr[col];
			}
			for (int i=0; i<number_of_scanline_rxs; ++i) {
				sendto(udp_sockfd, scanOut, 848*2*2, 0,
					(const struct sockaddr*)&scanline_rx_addr[i], sizeof(scanline_rx_addr[i]));
			}


#ifdef USE_LOCAL_DISPLAY
			cv::imshow("RealSense", out_image);
			cv::waitKey(1);
#endif

			ts1 = gettimeofday_as_double();
			tFrame = ts1 - ts0;
			ts0 = ts1;

			fps = 0.667 * fps + 0.333 * (1.0 / tFrame);
			printf("%.1f \n", fps);

		}
	}

	return 0;
}


