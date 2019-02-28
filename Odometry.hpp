#ifndef __ODOMETRY__HPP_
#define __ODOMETRY__HPP_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


class Odometry
{
public:
	Odometry(const rs2_intrinsics, float depth_scale, int sockfd);
	void opticalFlow(cv::Mat color_image, cv::Mat depth_image);
	void updatePointCloud();
	void sendPointcloudOverNetwork();
	void findOdometry(int camNumber);
	void draw_lines_and_stuff(cv::Mat color_image);

	double _speedX = 0.0;
	double _speedZ = 0.0;
private:
	int _frame_idx = 0;
	int _sockfd;
	struct sockaddr_in _rxAddr;

	struct timeval _tv;
	double _X = 0.0;
	double _Z = 0.0;

	// How to find points in 3-D space:
	rs2_intrinsics _rgb_intrinsics;
	float _depth_scale;

	cv::Mat _gray_image;
	cv::Mat _prev_gray_image;

	std::vector<std::vector<cv::Point2f>> _tracks;
	std::vector<std::vector<cv::Point2f>> _new_tracks;

	std::vector<std::vector<cv::Point3f>> _cloud_points;
	std::vector<std::vector<cv::Point3f>> _new_cloud_points;

	//std::vector<cv::Point2f> _prev_points;
	//std::vector<cv::Point2f> _cur_points;
	cv::Mat_<double> _prev_points;
	cv::Mat_<double> _cur_points;
};

#endif // __ODOMETRY__HPP_

