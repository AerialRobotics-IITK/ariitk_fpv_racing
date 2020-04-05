#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <vector>
#include <utility>

namespace ariitk::detector {

class Detector {
    public : 
        Detector() {};
        ~Detector();

        std::pair<int, int> getCentre() { return centre_;};
            
        void setHSVMin(const int& h , const int& s , const int& v) {
            hsv_min_ = cv::Scalar(h,s,v);
        }
        void setHSVMax(const int& h , const int& s , const int& v) {
            hsv_max_ = cv::Scalar(h,s,v);
        }

        void setMinArea(const int& a) {
            min_contour_area_ = a;
        }

        void setCannyParams(const int&, const int& , const int& );

        void thresholdImage(cv::Mat& );
        void findGoodContours();
        void drawContours(cv::Mat&);
        void findFrameCentre(cv::Mat&);
    
        cv::Mat getThresh() { return thresh_img_; };

    private :
        std::pair<int, int> centre_;
    
        cv::Scalar hsv_min_;
        cv::Scalar hsv_max_;
    
        cv::Mat thresh_img_;
    
        std::vector<std::vector<cv::Point>> good_contours_; 
    
        int canny_param_low_;
        int canny_param_upper_;
        int canny_kernel_size_;

        double min_contour_area_;
};

}   // namespace ariitk::detector
