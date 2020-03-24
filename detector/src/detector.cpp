#include <detector/detector.hpp>

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
        hsv_min_ = cv::Scalar(h,s,v);
    }

    void thresholdImage(cv::Mat& img) {
        cv::GaussianBlur( img, img, cv::Size(3,3), 0, 0 );
        cv::cvtColor( img, img, CV_BGR2HSV);
        cv::inRange( img, hsv_min_, hsv_max_);
        /*cv::dilate( img, img, cv::Mat(), cv::Point(-1,-1), 2, 1, 1);
        cv::erode( img ,img, cv::Mat(), cv::Point(-1,-1), 2, 1, 1);*/
        thresh_img_ = img;
    }

    void findGoodContours() {
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

    void drawContours(cv::Mat& test) {
        int id;
        int size = good_contours_.size();
        for(id=0; id<size ; id+=1){
            cv::drawContours(test, good_contours_, id, cv::Scalar(255,255,255), 3 );
        } 
        cv::imshow("Test Contour Image", test);
        cv::waitKey(0);
        cv::destroyWindow("Test Contour Image");
    }

    void findFrameCentre() {
        int i, size, x, y;
        float l;
        long int area=-1;
        long int max_area=-1;
        std::vector<cv::Point> approx;
        std::vector<std::vector<cv::Point>>::iterator ptr = good_contours_.begin();
        std::vector<std::vector<cv::Point>>::iterator end = good_contours_.end();
        for( ptr; ptr!=end; ++ptr){
            cv::approxPolyDP(*ptr,approx, 0.02*cv::arcLength(good_contours_[i], true), true);
            size = approx.size();
            if(size==4){
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
                    centre_.first = x;
                    centre_.second = y;
                }
            }
        }
    }
}