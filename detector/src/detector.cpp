#include <detector/detector.hpp>
#include <iostream>

namespace ariitk::detector {

Detector::Detector() {
    hsv_min_ = cv::Scalar(100,110,110);
    hsv_max_ = cv::Scalar(179,255,255);
    canny_param_low_ = 200;
    canny_param_upper_ = 300;
    canny_kernel_size_ = 3;
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
    if (img.empty()){return;};
        
    cv::GaussianBlur( img, img, cv::Size(3,3), 0, 0 );
    cv::cvtColor( img, img, CV_BGR2HSV);
    cv::inRange( img, hsv_min_, hsv_max_, thresh_img_);
    /*cv::dilate( img, img, cv::Mat(), cv::Point(-1,-1), 2, 1, 1);
    cv::erode( img ,img, cv::Mat(), cv::Point(-1,-1), 2, 1, 1);*/
    //thresh_img_ = img;
}

void Detector::findGoodContours() {
        
    cv::Canny(thresh_img_ , thresh_img_, canny_param_low_, canny_param_upper_ , canny_kernel_size_);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresh_img_, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>>::iterator ptr = contours.begin();
    std::vector<std::vector<cv::Point>>::iterator end = contours.end();
    for(ptr;ptr!=end;++ptr){
        if(cv::contourArea(*ptr)>500)
            good_contours_.push_back(*ptr);
    }
}

void Detector::drawContours(cv::Mat& test) {
    int id;
    int size = good_contours_.size();
    for(id=0; id<size ; id+=1){
        cv::drawContours(test, good_contours_, id, cv::Scalar(255,255,255), 3 );
    } 
}

void Detector::findFrameCentre(cv::Mat& test) {
    int i, size, x, y, id;
    float l;
    long int area=-1;
    long int max_area=-1;

    centre_.first = -1;
    centre_.second = -1;
    std::vector<cv::Point> approx;
    std::vector<std::vector<cv::Point>>::iterator ptr = good_contours_.begin();
    std::vector<std::vector<cv::Point>>::iterator end = good_contours_.end();
    for( ptr, id=0; ptr!=end; ++ptr, id+=1){
        cv::approxPolyDP(*ptr,approx, 0.02*cv::arcLength(*ptr, true), true);
        size = approx.size();
            
        if(size==4){
                
            cv::drawContours(test, good_contours_, id, cv::Scalar(255,255,255), 3);
            x=0;
            y=0;
            area = cv::contourArea(*ptr);
            if(area>=max_area){
                max_area = area;
                l=0.0;
                for(i=0;i<size;i+=1){
                    x = x + approx[i].x;
                    y = y + approx[i].y;
                }
                x = x/size;
                y = y/size;
                cv::circle(test,cv::Point(x,y),5,cv::Scalar(0,0,255),-1);
                centre_.first = x;
                centre_.second = y;
            }
        }
    }
    std::cout<<centre_.first<<" , "<<centre_.second<<std::endl;
}
}