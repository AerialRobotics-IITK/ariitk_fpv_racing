#include <detector/detector.hpp>
#include <iostream>

namespace ariitk::detector {

Detector::~Detector() { cv::destroyAllWindows(); }

double Detector::scalef = 77 * 4.2;
void Detector::setCannyParams(const int& l, const int& u, const int& s) {
	canny_param_low_ = l;
	canny_param_upper_ = u;
	canny_kernel_size_ = s;
}

void Detector::thresholdImage(cv::Mat& img) {
	if (img.empty()) { return; }

	cv::Mat blur_img(img.size(), CV_8UC3);
	cv::Mat hsv_img(img.size(), CV_8UC3);

	cv::GaussianBlur(img, blur_img, cv::Size(3, 3), 0, 0);
	cv::cvtColor(blur_img, hsv_img, CV_BGR2HSV);
	cv::inRange(hsv_img, hsv_min_, hsv_max_, thresh_img_);
}

void Detector::findGoodContours() {
	cv::Mat canny_img(thresh_img_.size(), CV_8UC1);

	cv::Canny(thresh_img_, canny_img, canny_param_low_, canny_param_upper_, canny_kernel_size_);

	good_contours_.clear();
	good_contours_.shrink_to_fit();
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(thresh_img_, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	good_contours_.clear();
	for (auto& contour : contours) {
		if (cv::contourArea(contour) > min_contour_area_) { good_contours_.push_back(contour); }
	}
}

void Detector::drawContours(cv::Mat& board) { cv::drawContours(board, good_contours_, -1, cv::Scalar(255, 255, 255), 3); }

void Detector::findFrameCentre(cv::Mat& board) {
	long int area = -1;
	long int max_area = -1;

	centre_.first = -1;
	centre_.second = -1;
	distance_ = 0.0;

	std::vector<cv::Point> approx;
	std::vector<std::vector<cv::Point>> contour_to_draw;

	for (auto& contour : good_contours_) {
		cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);

		if (approx.size() == 4) {
			area = cv::contourArea(contour);

			contour_to_draw.clear();
			contour_to_draw.push_back(contour);
			cv::drawContours(board, contour_to_draw, -1, cv::Scalar(0, 0, 255), 3);

			centre_.first = 0;
			centre_.second = 0;

			if (area >= max_area) {
				max_area = area;

				for (auto& corner : approx) {
					centre_.first += corner.x;
					centre_.second += corner.y;
				}
				centre_.first /= 4;
				centre_.second /= 4;
				distance_ =
				    sqrt((approx[1].x - approx[0].x) * (approx[1].x - approx[0].x) + (approx[1].y - approx[0].y) * (approx[1].y - approx[0].y));
				distance_ = scalef / distance_;
				cv::circle(board, cv::Point(centre_.first, centre_.second), 5, cv::Scalar(0, 255, 0), -1);
			} else {
				centre_.first = -1;
				centre_.second = -1;
			}
		}
	}
}

} // namespace ariitk::detector
