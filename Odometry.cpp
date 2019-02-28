
#include <string>
#include <iostream>
#include <math.h>
#include "Odometry.hpp"


Odometry::Odometry(const rs2_intrinsics rgb_intrinsics, float depth_scale, int sockfd) {

	_rgb_intrinsics.width	 = rgb_intrinsics.width;
	_rgb_intrinsics.height	= rgb_intrinsics.height;
	_rgb_intrinsics.ppx	   = rgb_intrinsics.width;
	_rgb_intrinsics.ppy	   = rgb_intrinsics.height;
	_rgb_intrinsics.fx		= rgb_intrinsics.fx;
	_rgb_intrinsics.fy		= rgb_intrinsics.fy;
	_rgb_intrinsics.model	 = rgb_intrinsics.model;
	_rgb_intrinsics.coeffs[0] = rgb_intrinsics.coeffs[0];
	_rgb_intrinsics.coeffs[1] = rgb_intrinsics.coeffs[1];
	_rgb_intrinsics.coeffs[2] = rgb_intrinsics.coeffs[2];
	_rgb_intrinsics.coeffs[3] = rgb_intrinsics.coeffs[3];
	_rgb_intrinsics.coeffs[4] = rgb_intrinsics.coeffs[4];

	_depth_scale = depth_scale;
	_sockfd = sockfd;

	_rxAddr.sin_family = AF_INET;
	_rxAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	_rxAddr.sin_port = htons(9221);

	gettimeofday(&_tv, NULL);
}



void Odometry::opticalFlow(cv::Mat color_image, cv::Mat depth_image) {

	// LK parameters (to track existing points)
	cv::Size winSize(15, 15);
	int maxLevel = 4;
	cv::TermCriteria termcrit(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 0.03);
	//double minEigThreshold = 3.0e-4;

	// Feature parameters (to detect new dots)
	int maxCorners = 500;
	double qualityLevel = 0.04;
	double minDistance = 7.0;
	int blockSize = 7;

	cv::Mat mask;

	int track_len = 10;
	int frame_idx = 0;
	int detect_interval = 3;

	std::vector<cv::Point2f> p0;
	std::vector<cv::Point2f> p1;
	std::vector<cv::Point2f> p0r;
	std::vector<cv::Point2f> pNew;

	cv::cvtColor(color_image, _gray_image, cv::COLOR_BGR2GRAY);

	if (_prev_gray_image.empty()) {
		_gray_image.copyTo(_prev_gray_image);
	}
	if (!_tracks.empty()) {

		std::vector<uchar> status;
		std::vector<float> err;

		// The p0 vector is just the last point in each of the tracks items
		p0.resize(_tracks.size());
		for (int i=0; i<_tracks.size(); ++i) {
			p0[i] = _tracks[i].back();
		}

		// ********************************
		// Lucas-Kanade Sparse Optical Flow
		// ********************************

		// Forward tracking:
		cv::calcOpticalFlowPyrLK(_prev_gray_image, _gray_image, p0, p1, status, err,
			winSize, maxLevel, termcrit);

		// Reverse tracking:
		cv::calcOpticalFlowPyrLK(_gray_image, _prev_gray_image, p1, p0r, status, err,
			winSize, maxLevel, termcrit);


		_new_tracks.resize(0);
		_new_cloud_points.resize(0);

		for (int i=0; i<p0.size(); ++i) {

			// If the distance from forward to reverse tracking is about a single pixel,
			// then it's a good track.  Otherwise, discard.
			auto d = cv::norm(p0[i] - p0r[i]);
			if (p1[i].x < 0 || p1[i].x > 847 || p1[i].y < 0 || p1[i].y > 479) {
				continue;
			}
			float z_depth = _depth_scale * depth_image.at<uint16_t>(p1[i]);
			// the RealSense D435 has a usable range between 0.2m and 8.0m; anything
			// else is BS
			if ((d < 2.0) && (z_depth > 0.2) && (z_depth < 8.0)) {

				_tracks[i].push_back(cv::Point2f(p1[i].x, p1[i].y));
				while (_tracks[i].size() > track_len) {
					_tracks[i].erase(_tracks[i].begin());
				}
				_new_tracks.push_back(_tracks[i]);

				float pt3d[3];
				float pixel[2] = {p1[i].x, p1[i].y};
				rs2_deproject_pixel_to_point(pt3d, &_rgb_intrinsics, pixel, z_depth);

				_cloud_points[i].push_back(cv::Point3f(pt3d[0], -pt3d[1], -pt3d[2]));
				while (_cloud_points[i].size() > track_len) {
					_cloud_points[i].erase(_cloud_points[i].begin());
				}
				_new_cloud_points.push_back(_cloud_points[i]);
			}
		}

		std::swap(_new_tracks, _tracks);
		std::swap(_new_cloud_points, _cloud_points);
	}

	// Every once-in-while, we'll try to add new points to the list of
	// points that we're tracking:
	if (frame_idx % detect_interval == 0 && _tracks.size() < 300) {

		// The p0 vector is just the last point in each of the tracks items
		p0.resize(_tracks.size());
		for (int i=0; i<_tracks.size(); ++i) {
			p0[i] = _tracks[i].back();
		}

		// We won't bother detecting near points that we're already tracking:
		mask.create(_gray_image.size(), _gray_image.type());
		mask.setTo(255);
		for (int i=0; i<p0.size(); ++i) {
			cv::circle(mask, p0[i], 5, 0, -1);
		}

		cv::goodFeaturesToTrack(_gray_image, pNew,
			maxCorners, qualityLevel, minDistance, mask, blockSize); //, 3, 0, 0.04);

		for (int i=0; i<pNew.size(); ++i) {

			float z_depth = _depth_scale * depth_image.at<uint16_t>(pNew[i]);
			// the RealSense D435 has a usable range between 0.2m and 8.0m; anything
			// else is BS
			if (z_depth > 0.2 && z_depth < 8.0) {

				std::vector<cv::Point2f> newTrack;
				newTrack.push_back(cv::Point2f(pNew[i].x, pNew[i].y));
				_tracks.push_back(newTrack);

				//_tracks[i].push_back(cv::Point2f(pNew[i].x, pNew[i].y));
				//if (_tracks[i].size() > track_len) {
				//	_tracks[i].erase(_tracks[i].begin());
				//}

				float pt3d[3];
				float pixel[2] = {pNew[i].x, pNew[i].y};
				rs2_deproject_pixel_to_point(pt3d, &_rgb_intrinsics, pixel, z_depth);

				std::vector<cv::Point3f> newPoints;
				newPoints.push_back(cv::Point3f(pt3d[0], -pt3d[1], -pt3d[2]));
				_cloud_points.push_back(newPoints);
				//if (_cloud_points[i].size() > track_len) {
				//	_cloud_points[i].erase(_cloud_points[i].begin());
				//}
			}
		}
	}

	frame_idx++;
	cv::swap(_prev_gray_image, _gray_image);

}


void Odometry::updatePointCloud() {
	int numPts = 0;
	for (int i=0; i<_cloud_points.size(); ++i) {
		if (_cloud_points[i].size() > 1) {
			numPts++;
		}
	}
	_prev_points.create(numPts, 3);
	_cur_points.create(numPts, 3);



	for (int i=0; i<_cloud_points.size(); ++i) {
		int n = _cloud_points[i].size();
		if (n > 1) {
			_prev_points(i, 0) = _cloud_points[i][n-2].x;
			_prev_points(i, 1) = _cloud_points[i][n-2].y;
			_prev_points(i, 2) = _cloud_points[i][n-2].z;
			_cur_points(i, 0) = _cloud_points[i][n-1].x;
			_cur_points(i, 1) = _cloud_points[i][n-1].y;
			_cur_points(i, 2) = _cloud_points[i][n-1].z;
		}
	}
}


void Odometry::sendPointcloudOverNetwork() {
	float outBuf[8000];
	float outBuf2[8000];
	//size_t size = sizeof(float) * 3 * numPts;
	int i;

	for (i=0; i<_cloud_points.size(); ++i) {
		int n = _cloud_points[i].size();
		if (n > 1) {
			outBuf[i*3 + 0] = _cloud_points[i][n-1].x;
			outBuf[i*3 + 1] = _cloud_points[i][n-1].y;
			outBuf[i*3 + 2] = _cloud_points[i][n-1].z;

			outBuf2[i*3 + 0] = _cloud_points[i][n-2].x;
			outBuf2[i*3 + 1] = _cloud_points[i][n-2].y;
			outBuf2[i*3 + 2] = _cloud_points[i][n-2].z;

			if (i > 2000) {
				break;
			}
		}
	}

	_rxAddr.sin_port = htons(9221);
	sendto(_sockfd, (char*)outBuf, i*12, 0,
			(const struct sockaddr*) &_rxAddr, sizeof(struct sockaddr));
	_rxAddr.sin_port = htons(9222);
	sendto(_sockfd, (char*)outBuf2, i*12, 0,
			(const struct sockaddr*) &_rxAddr, sizeof(struct sockaddr));

}





cv::Vec3d CalculateMean(const cv::Mat &points) {
	// Based on:  https://stackoverflow.com/questions/21206870
	// and:  http://nghiaho.com/?page_id=671

	cv::Mat_<cv::Vec3d> result;
	cv::reduce(points, result, 0, CV_REDUCE_AVG);
	return result(0, 0);
}



cv::Mat_<double> FindRigidTransform(
		const cv::Mat_<cv::Vec3d> A,
		const cv::Mat_<cv::Vec3d> B) {
	// Based on:  https://stackoverflow.com/questions/21206870
	// and:  http://nghiaho.com/?page_id=671

	cv::Vec3d centroid_A = CalculateMean(A);
	cv::Vec3d centroid_B = CalculateMean(B);

	cv::Mat_<double> T1 = cv::Mat_<double>::eye(4, 4);
	T1(0, 3) = -centroid_A[0];
	T1(1, 3) = -centroid_A[1];
	T1(2, 3) = -centroid_A[2];

	cv::Mat_<double> T2 = cv::Mat_<double>::eye(4, 4);
	T2(0, 3) = centroid_B[0];
	T2(1, 3) = centroid_B[1];
	T2(2, 3) = centroid_B[2];

	// Dot product
	cv::Mat_<double> C(3, 3, 0.0);
	for (int ptIdx = 0; ptIdx < A.rows; ++ptIdx) {
		cv::Vec3d p1 = A(ptIdx, 0) - centroid_A;
		cv::Vec3d p2 = B(ptIdx, 0) - centroid_B;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				C(i, j) += p2[i] * p1[j];
			}
		}
	}

	cv::Mat_<double> u, s, vt;
	cv::SVD::compute(C, s, u, vt);

	cv::Mat_<double> R = u * vt;


	// special reflection case
	if (cv::determinant(R) < 0) {
		printf("Reflection detected\n");
		R -= u.col(2) * (vt.row(2) * 2.0);
	}

	cv::Mat_<double> M = cv::Mat_<double>::eye(4, 4);
	R.copyTo(M.colRange(0, 3).rowRange(0, 3));

	cv::Mat_<double> result = T2 * M * T1;

	return result;
}

void Odometry::findOdometry(int camNumber) {
	if (_prev_points.rows > 5 && _cur_points.rows > 5) {
		//printf("odo, cam#%d\n", camNumber);

		if (_prev_points.rows != _cur_points.rows) {
			printf(" WRONG ROW SIZE!\n");
		}
		if (_prev_points.cols != _cur_points.cols) {
			printf(" WRONG ROW SIZE!\n");
		}
		struct timeval tv1;
		struct timeval delta_tv;

		cv::Mat_<double> xfrm = FindRigidTransform(_prev_points, _cur_points);
		//std::cout << xfrm << std::endl;

		double X = xfrm.at<double>(0,3);
		double Z = xfrm.at<double>(2,3);
		if (!std::isnan(X) && !std::isnan(Z)) {
			gettimeofday(&tv1, NULL);
			timersub(&tv1, &_tv, &delta_tv);
			_tv.tv_sec = tv1.tv_sec;
			_tv.tv_usec = tv1.tv_usec;
			double deltaT = delta_tv.tv_sec + delta_tv.tv_usec / 1000000.0;
			double speedX = X / deltaT;
			double speedZ = Z / deltaT;
			_speedX = 0.66 * _speedX + 0.34 * speedX;
			_speedZ = 0.66 * _speedZ + 0.34 * speedZ;

			//_X += X;
			//_Z += Z;
			//printf("%d  X: %.03f   Z: %.03f\n", camNumber, _speedX, _speedZ);
			//printf("X: %.03f    Z: %.03f\n", _X, _Z);
		}
	}
}



void Odometry::draw_lines_and_stuff(cv::Mat color_image) {

	for (int i=0; i<_tracks.size(); ++i) {
		for (int j=0; j<_tracks[i].size() - 1; ++j) {
			cv::line(color_image, _tracks[i][j], _tracks[i][j+1], cv::Scalar(0, 255, 0));
		}
		// Why this no worky?:
		//cv::polylines(color_image, _tracks[i], false, cv::Scalar(0,255,0));

		cv::circle(color_image, _tracks[i].back(), 2, cv::Scalar(0,255,0), -1);
	}
}



