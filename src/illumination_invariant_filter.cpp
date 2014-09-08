#include <image_utils/illumination_invariant_filter.h>
#include <fastonebigheader.h>

PLUGINLIB_DECLARE_CLASS(image_utils, IlluminationInvariantFilter, image_utils::IlluminationInvariantFilter, nodelet::Nodelet)

namespace enc = sensor_msgs::image_encodings;

namespace image_utils
{

    void IlluminationInvariantFilter::onInit(){
        ros::NodeHandle& nh = getNodeHandle();
        it_.reset(new image_transport::ImageTransport(nh));

        auto connect_cb = boost::bind(&IlluminationInvariantFilter::connectCb, this);
        // lock so connectCb is not called between advertising and assigning pub_illumination_invariant_
        std::lock_guard<std::mutex> lock(connect_mutex_);
        pub_illumination_invariant_ = it_->advertise("image_mono_invariant", 1, connect_cb, connect_cb);

    }

    void IlluminationInvariantFilter::connectCb(){
        std::lock_guard<std::mutex> lock(connect_mutex_);
        // if we got a connect callback and there are no subscribers, shutdown 
        if (pub_illumination_invariant_.getNumSubscribers() == 0)
            sub_raw_.shutdown();
        else if(!sub_raw_){
            image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
            sub_raw_ = it_->subscribe("image_raw", 1, &IlluminationInvariantFilter::imageCb, this, hints);
        }
    }

    void IlluminationInvariantFilter::imageCb(const sensor_msgs::ImageConstPtr& raw_msg){
        if (pub_illumination_invariant_.getNumSubscribers()){
            // get pointer to cv Mat
            auto raw_ptr = cv_bridge::toCvShare(raw_msg, raw_msg->encoding);
            // get illumination invariant image
            transformImage(raw_ptr->image, illumination_invariant_image_);
            // publisht the new image
            sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(raw_msg->header, enc::MONO8, illumination_invariant_image_).toImageMsg();
            pub_illumination_invariant_.publish(output_msg);
        }
    }

    void IlluminationInvariantFilter::transformImage(const cv::Mat &image, cv::Mat &result){
        result = cv::Mat::zeros(image.rows,image.cols, CV_8UC1);
        static const float c = cos(2.3);
        static const float s = sin(2.3);
        static const float one_third = 1.0/3.0;
        float R=0,G=0,B=0;
        float RGB=0,r=0,b=0;
        int cols = image.cols;
        int rows = image.rows;
        uchar* result_ptr = (uchar*)result.data;

        for(int i = 0; i < rows*cols; i++) {
            R = (uchar) image.data[i*3];
            G = (uchar) image.data[i*3 + 1];
            B = (uchar) image.data[i*3 + 2];

            R =  0.998800*R - 0.066900*G - 0.000100*B;
            G = -0.049700*R + 0.988600*G - 0.000100*B;
            B =  0.004300*R - 0.134600*G + 1.000000*B;

            if ( R > 245 ){ R = 245; } else if ( R < 10 ){ R = 10; }
            if ( G > 245 ){ G = 245; } else if ( G < 10 ){ G = 10; }
            if ( B > 245 ){ B = 245; } else if ( B < 10 ){ B = 10; }

            RGB = R*G*B;
            b = fasterlog(R) - one_third*fasterlog(RGB);
            r = fasterlog(B) - one_third*fasterlog(RGB);

            // magic numbers for linear stretching of the dynamic range
            *result_ptr = (uchar)(32.0*(r*c+b*s) +128.0);
            result_ptr++;
        }
        int ksize=5;
        cv::GaussianBlur(result,result,cv::Size(ksize,ksize),0,0);
    }
}

