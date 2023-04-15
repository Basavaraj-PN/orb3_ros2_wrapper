#include "adapters.hpp"
#include "helper_functions.hpp"

namespace CAMERA
{
    void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {
        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptr;

        header = img_msg->header;
        try
        {
            cv_ptr = cv_bridge::toCvShare(img_msg, "mono8");
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("logger"), "cv_bridge exception: %s", e.what());
            return;
        }
        Tcc = mpSLAM->TrackMonocular(cv_ptr->image, timestamp_to_sec(cv_ptr->header));
    }

} // CAMERA namespace

namespace IMU
{

    void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
    {
        mBufMutex.lock();
        imuBuf.push(imu_msg);
        mBufMutex.unlock();

        return;
    }
} // IMU namespace
