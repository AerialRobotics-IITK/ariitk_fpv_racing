#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <utility>
#include <iterator>

namespace ariitk::detector {
    class Detector {
        public : 
            Detector();
            ~Detector();
            std::pair<int, int> getCentre() { return centre_;};
            void setHSVMin(const int& , const int& , const int&);
            void setHSVMax(const int& , const int& , const int&);
            void thresholdImage(cv::Mat& );
            void findGoodContours();
            void drawContours(cv::Mat&);
            void findFrameCentre();

        private :
            std::pair<int, int> centre_;
            cv::Scalar hsv_min_;
            cv::Scalar hsv_max_;
            cv::Mat thresh_img_;
            std::vector<std::vector<cv::Point>> good_contours_; 
            int canny_param_low_;
            int canny_param_upper_;
            int canny_kernel_size_;
    };
}