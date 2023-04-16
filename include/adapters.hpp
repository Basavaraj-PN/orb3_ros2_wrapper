#include "common.hpp"

namespace IMU
{
    /**
     * @brief Class for grabbing IMU data.
     */
    class ImuGrabber
    {
    public:
        /**
         * @brief Default constructor for ImuGrabber.
         */
        ImuGrabber(){};

        /**
         * @brief Method to grab IMU data.
         * @param imu_msg Shared pointer to the sensor_msgs::msg::Imu ROS message.
         */
        void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);
        queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf; /**< Queue to store IMU messages. */
        std::mutex mBufMutex;                                /**< Mutex for thread-safe access to imuBuf. */
    };
} // IMU namespace

namespace CAMERA
{ /**
   * @brief Class for grabbing camera image data.
   */
    class ImageGrabber
    {
    public:
        /**
         * @brief Constructor for ImageGrabber.
         * @param pSLAM Pointer to the ORB_SLAM3::System object.
         */
        ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}
        /**
         * @brief Method to grab camera image data.
         * @param img_msg Shared pointer to the sensor_msgs::msg::Image ROS message.
         */
        void GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
        ORB_SLAM3::System *mpSLAM;    /**< Pointer to the ORB_SLAM3::System object. */
        Sophus::SE3f Tcc;             /**< Camera pose. */
        std_msgs::msg::Header header; /**< ROS message header, to fetch stamp and frame_id */
    };

} // CAMERA namespace
