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

#ifdef USE_ZBAR
#include <string>
#include <iostream>

#include <zbar.h>
#endif // USE_ZBAR

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

void printTimeofday(std::string prefix) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	printf("%s%ld.%06ld\n", prefix.c_str(), tv.tv_sec, tv.tv_usec);
}



void get_frame_thread(rs2::frameset frames, int cam_number) {

	rs2::align align(RS2_STREAM_COLOR);
	auto processed = align.process(frames);
	rs2::video_frame vid_frame = processed.get_color_frame();
	rs2::depth_frame dep_frame = processed.get_depth_frame();

	cv::Mat c_image(cv::Size(vid_frame.get_width(), vid_frame.get_height()),
		CV_8UC3, (void*)vid_frame.get_data(), cv::Mat::AUTO_STEP);

	cv::Mat d_image(cv::Size(dep_frame.get_width(), dep_frame.get_height()),
		CV_16UC1, (void*)dep_frame.get_data(), cv::Mat::AUTO_STEP);

	if (cam_number == 0) {
		// camera 0 is upside-down
		cv::flip(c_image, color_image[0], -1);
		cv::flip(d_image, depth_image[0], -1);
	} else {
		// TODO:  how to keep the actual data in scope without doing a copy...
		c_image.copyTo(color_image[1]);
		d_image.copyTo(depth_image[1]);
	}

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


#ifdef USE_ZBAR

zbar::Image *zImage[2];
int num_qr_codes[2];

void QR_code_detect_thread(int cam_number) {
	zbar::ImageScanner zbarScanner;
	zbarScanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
	zbarScanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

	// TODO:  not entirely sure if I'm doing this right, or what the best setting is:
	// (I'm pretty sure that 1 is highest density/slowest speed)
	zbarScanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_X_DENSITY, 1);
	zbarScanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_Y_DENSITY, 1);


	if (out_image.cols > 10 && out_image.rows > 10) {
		cv::Mat gray_image1;
		cv::cvtColor(color_image[cam_number], gray_image1, cv::COLOR_RGB2GRAY);

		zImage[cam_number] = new zbar::Image(gray_image1.cols, gray_image1.rows, "Y800", (uchar *)gray_image1.data, gray_image1.cols * gray_image1.rows);

		int n = zbarScanner.scan(*zImage[cam_number]);
		num_qr_codes[cam_number] = n;
	}
}
#endif // USE_ZBAR



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

	printf("left_camera_serial: %s\n", left_camera_serial.c_str());
	printf("right_camera_serial: %s\n", right_camera_serial.c_str());


#ifdef USE_NETWORK_DISPLAY
	struct sockaddr_in jpgRxAddr;
	jpgRxAddr.sin_family = AF_INET;
	jpgRxAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	jpgRxAddr.sin_port = htons(3123);
	if (fs["network_jpg_address"].type() == cv::FileNode::STR) {
		jpgRxAddr.sin_addr.s_addr = inet_addr(fs["network_jpg_address"].string().c_str());
	}
	if (fs["network_jpg_port"].type() == cv::FileNode::INT) {
		jpgRxAddr.sin_port = htons(fs["network_jpg_port"].real());
	}

	char buffer[24];
	inet_ntop(AF_INET, &(jpgRxAddr.sin_addr), buffer, 24);
	printf("\nSending network video to: %s %d/udp\n\n", buffer, ntohs(jpgRxAddr.sin_port));
#endif // USE_NETWORK_DISPLAY


	struct sockaddr_in *scanline_rx_addr;
	int number_of_scanline_rxs = 0;
	if (fs["scanline_rx"].type() == cv::FileNode::SEQ) {
		number_of_scanline_rxs = fs["scanline_rx"].size();
		scanline_rx_addr = (struct sockaddr_in*)malloc(number_of_scanline_rxs* sizeof(struct sockaddr_in));
		for (int i=0; i< number_of_scanline_rxs; ++i) {
			std::string addr = fs["scanline_rx"][i]["addr"].string();
			int port = fs["scanline_rx"][i]["port"].real();
			printf("Sending scanline data to: %s %d/udp\n", addr.c_str(), port);

			scanline_rx_addr[i].sin_family = AF_INET;
			scanline_rx_addr[i].sin_addr.s_addr = inet_addr(addr.c_str());
			scanline_rx_addr[i].sin_port = htons(port);
		}
	}
	printf("\n");


	bool USE_LOCAL_DISPLAY = false;
	if (fs["use_local_display"].type() == cv::FileNode::INT) {
		if (fs["use_local_display"].real() == 1) {
			USE_LOCAL_DISPLAY = true;
		}
	}


	fs.release();

	// ##########################################################
	// ##########################################################
	//     END:  Read in config file
	// ##########################################################
	// ##########################################################


	int udp_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (udp_sockfd < 0) {
		perror("failed to open UDP socket");
		exit(1);
	}

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

	const rs2_intrinsics rgb_intrs[2] = {
		rgb_intrinsics_0a, rgb_intrinsics_1
	};

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

	while (true) {

#ifdef USE_NETWORK_DISPLAY
		std::thread t2(network_jpg_sender_thread, udp_sockfd, out_image, jpgRxAddr);
#endif

#ifdef USE_ZBAR
		std::thread t3(QR_code_detect_thread, 0);
		std::thread t4(QR_code_detect_thread, 1);
#endif // USE_ZBAR

		rs2::frameset frames_0 = pipeline_0.wait_for_frames();
		rs2::frameset frames_1 = pipeline_1.wait_for_frames();

		std::thread t0(get_frame_thread, frames_0, 0);
		std::thread t1(get_frame_thread, frames_1, 1);


		t0.join();
		t1.join();

#ifdef USE_NETWORK_DISPLAY
			t2.join();
#endif

#ifdef USE_ZBAR
		t3.join();
		t4.join();

		for (int camNo=0; camNo<2; ++camNo) {
			if (num_qr_codes[camNo] > 0) {

				zbar::Image::SymbolIterator symbol = zImage[camNo]->symbol_begin();
				for(; symbol != zImage[camNo]->symbol_end(); ++symbol)
				{
					//printf("type: %s, data: %s\n", symbol->get_type_name(), symbol->get_data());
					std::cout << "Type: " << symbol->get_type_name() << std::endl;
					std::cout << "Data: " << symbol->get_data() << std::endl;
					float avg_point[3] = {0,0,0};
					bool _valid = true;
					for(int j=0; j < symbol->get_location_size(); ++j) {
						float pixel[2] = {symbol->get_location_x(j), symbol->get_location_y(j)};
						cv::Point pt(pixel[0], pixel[1]);

						cv::circle(color_image[camNo], pt, 6, cv::Scalar(0,0,255), -1);

						float point[3];

						uint16_t _d = depth_image[camNo].at<uint16_t>(pixel[1], pixel[0]);
						float depth = _d * depth_scale;
						if (depth < 8.0) {
							rs2_deproject_pixel_to_point(point, &rgb_intrs[camNo], pixel, depth);
							avg_point[0] += point[0] / symbol->get_location_size();;
							avg_point[1] += point[1] / symbol->get_location_size();;
							avg_point[2] += point[2] / symbol->get_location_size();;
						} else {
							_valid = false;
						}
					}
					if (_valid) {
						printf("%f, %f, %f \n", avg_point[0], avg_point[1], avg_point[2]);
					}
				}
			}
		}
#endif // USE_ZBAR

		if (color_image[0].rows > 10 && color_image[0].rows == color_image[1].rows) {

			cv::hconcat(color_image[0], color_image[1], out_image);

			// http://longstryder.com/2014/07/which-way-of-accessing-pixels-in-opencv-is-the-fastest/
			// Get a scan of the very middle row (the horizon, in theory)
			const unsigned char *ptr;
			unsigned char scanOut[ 848 * 2 * 2 ];  // will be larger than ethernet MTU  :-( ...
			ptr = depth_image[0].ptr(240);
			for (int col=0; col < 848*2; ++col) {
				scanOut[col] = ptr[col];
			}
			ptr = depth_image[1].ptr(240);
			for (int col=0; col < 848*2; ++col) {
				scanOut[col+848*2] = ptr[col];
			}

			// send scanOut to target(s) via UDP:
			for (int i=0; i<number_of_scanline_rxs; ++i) {
				sendto(udp_sockfd, scanOut, 848*2*2, 0,
					(const struct sockaddr*)&scanline_rx_addr[i], sizeof(scanline_rx_addr[i]));
			}

			if (USE_LOCAL_DISPLAY) {
				cv::imshow("RealSense", out_image);
				cv::waitKey(1);
			}

			ts1 = gettimeofday_as_double();
			float tFrame = ts1 - ts0;
			ts0 = ts1;

			fps = 0.667 * fps + 0.333 * (1.0 / tFrame);
			printf("%.1f \n", fps);

		}
	}

	return 0;
}


