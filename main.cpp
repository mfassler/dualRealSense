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

#include <thread>

#include "librealsense2/rs.hpp"
#include "librealsense2/rsutil.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "UdpSender.hpp"

cv::Mat color_image[2];
cv::Mat depth_image[2];
cv::Mat out_image;


//#define USE_NETWORK_DISPLAY
#define USE_LOCAL_DISPLAY


double gettimeofday_as_double() {
	struct timeval tv;
	gettimeofday(&tv, NULL);

	double ts = tv.tv_sec;
	ts += (tv.tv_usec / 1000000.0);

	return ts;
}


void get_frame_thread(rs2::frameset frames, int cam_number) {

	rs2::align align(RS2_STREAM_COLOR);
	auto processed = align.process(frames);
	rs2::video_frame vid_frame = processed.get_color_frame();
	rs2::depth_frame dep_frame = processed.get_depth_frame();


	color_image[cam_number] = cv::Mat(cv::Size(vid_frame.get_width(), vid_frame.get_height()),
		CV_8UC3, (void*)vid_frame.get_data(), cv::Mat::AUTO_STEP);

	depth_image[cam_number] = cv::Mat(cv::Size(dep_frame.get_width(), dep_frame.get_height()),
		CV_16UC1, (void*)dep_frame.get_data(), cv::Mat::AUTO_STEP);

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



#ifdef USE_NETWORK_DISPLAY
UdpSender udpSender("192.168.100.103", 3123);

void network_sender_thread(std::vector<int> params) {
	std::vector<uchar> outputBuffer;

	if (out_image.cols > 10 && out_image.rows > 10) {
		if (!cv::imencode("*.jpg", out_image, outputBuffer, params)) {
			printf("failed to imencode\n");
		} else {
			udpSender.sendImage((unsigned char*)&outputBuffer[0], outputBuffer.size());
		}
	}
}
#endif // USE_NETWORK_DISPLAY


int main(int argc, char* argv[]) {
	std::vector<int> params;
	params.push_back(cv::IMWRITE_JPEG_QUALITY);
	params.push_back(30);

	rs2::pipeline pipeline_0;
	rs2::pipeline pipeline_1;

	rs2::config config_0;
	rs2::config config_1;

	config_0.enable_device("819612072741");
	config_0.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 30);
	config_0.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
	//config_0.enable_device_from_file(argv[1], false);


	config_1.enable_device("745212070261");
	config_1.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 30);
	config_1.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
	//config_1.enable_device_from_file(argv[2], false);

	auto profile_0 = pipeline_0.start(config_0);
	auto profile_1 = pipeline_1.start(config_1);

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
		std::thread t2(network_sender_thread, params);
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


