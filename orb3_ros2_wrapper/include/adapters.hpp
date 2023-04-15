#include "common.hpp"

namespace IMU
{
    class ImuGrabber
    {
    public:
        ImuGrabber(){};
        void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);
        queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;
        std::mutex mBufMutex;
    };
} // IMU namespace

namespace CAMERA
{
    class ImageGrabber
    {
    public:
        ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}
        void GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
        ORB_SLAM3::System *mpSLAM;
        Sophus::SE3f Tcc;
        std_msgs::msg::Header header;
    };

} // CAMERA namespace
