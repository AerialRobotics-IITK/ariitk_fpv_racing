#include <detector/detector.hpp>
#include <iostream>

namespace ariitk::detector {

Detector::Detector() {
    hsv_min_ = cv::Scalar(100,110,110);
    hsv_max_ = cv::Scalar(179,255,255);
    canny_param_low_ = 200;
    canny_param_upper_ = 300;
    canny_kernel_size_ = 3;
    min_contour_area_ = 500;
}

Detector::~Detector() {
    cv::destroyAllWindows();
}

void Detector::setHSVMin(const int& h , const int& s , const int& v) {
    hsv_min_ = cv::Scalar(h,s,v);
}

void Detector::setHSVMax(const int& h , const int& s , const int& v) {
    hsv_max_ = cv::Scalar(h,s,v);
}

void Detector::thresholdImage(cv::Mat& img) {
    if(img.empty()) { return; };
    
    cv::GaussianBlur(img, img, cv::Size(3,3), 0, 0);
    cv::cvtColor(img, img, CV_BGR2HSV);
    cv::inRange(img, hsv_min_, hsv_max_, thresh_img_);
}

void Detector::findGoodContours() {
    cv::Canny(thresh_img_ , thresh_img_, canny_param_low_, canny_param_upper_ , canny_kernel_size_);

    good_contours_.clear();
    good_contours_.shrink_to_fit();
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresh_img_, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    for(auto& contour : contours) { if(cv::contourArea(contour) > min_contour_area_) { good_contours_.push_back(contour); } }
}

void Detector::drawContours(cv::Mat& board) {
    cv::drawContours(board, good_contours_, -1, cv::Scalar(255,255,255), 3);
}

void Detector::findFrameCentre(cv::Mat& board) {
    long int area = -1;
    long int max_area = -1;

    centre_.first = -1;
    centre_.second = -1;

    std::vector<cv::Point> approx;
    std::vector<std::vector<cv::Point>> contour_to_draw;

    for(auto& contour : good_contours_) { 
        cv::approxPolyDP(contour, approx, 0.02*cv::arcLength(contour, true), true);
        
        if(approx.size() == 4) {
            area = cv::contourArea(contour);

            contour_to_draw.clear();
            contour_to_draw.push_back(contour);
            cv::drawContours(board, contour_to_draw, -1, cv::Scalar(255,255,255), 3);
            
            centre_.first = 0; centre_.second = 0;

            if(area >= max_area){
                max_area = area;

                for(auto& corner : approx) {
                    centre_.first += corner.x;
                    centre_.second += corner.y;
                }
                centre_.first /= 4; centre_.second /= 4;

                cv::circle(board, cv::Point(centre_.first, centre_.second), 5, cv::Scalar(255,255,255), -1);
            }
        }
    }
}

} // namespace ariitk::detector