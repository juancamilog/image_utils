#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <mutex>

namespace image_utils
{
    class IlluminationInvariantFilter: public nodelet::Nodelet
    {
        public:
            virtual void onInit();

            void connectCb();
            void imageCb(const sensor_msgs::ImageConstPtr& raw_msg);

            void transformImage(const cv::Mat &image, cv::Mat &result);

        private:
            boost::shared_ptr<image_transport::ImageTransport> it_;
            image_transport::Subscriber sub_raw_;
            image_transport::Publisher pub_illumination_invariant_;
            std::mutex connect_mutex_;
            cv::Mat illumination_invariant_image_;
    };
}
