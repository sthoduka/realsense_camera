#ifndef REALSENSE_CAMERA_NODELET_H_
#define REALSENSE_CAMERA_NODELET_H_

#include <map>

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "realsense_camera/realsenseConfig.h"
#include "realsense_camera/get_rgb_uv.h"

#include <dynamic_reconfigure/server.h>
#include <realsense_camera/RealsenseCameraConfig.h>

#include "capturer_mmap.h"


#define SHOW_RGBD_FRAME 0

#define USE_BGR24 0

typedef pcl::PointXYZ       PointXYZT;
typedef pcl::PointXYZRGB    PointXYZRGBT;
typedef pcl::PointXYZRGBA   PointXYZRGBAT;

typedef PointXYZRGBT PointType;

struct timeval start, all_start;
struct timeval end, all_end;
double timeuse, all_timeuse;

bool show_use_times = false;
//x: start timeval
#define USE_TIMES_START( x ) if(show_use_times){gettimeofday( &x, NULL );}

//x: start timeval
//y: end timeval
//z: show string
#define USE_TIMES_END_SHOW( x, y, z ) \
        if(show_use_times) \
        { \
            gettimeofday( &y, NULL ); \
            timeuse = 1000000 * ( y.tv_sec - x.tv_sec ) + y.tv_usec - x.tv_usec; \
            timeuse /= 1000000; \
            printf(z": [%f s]\n", timeuse); \
        }

namespace realsense_camera
{
    typedef pcl::PointXYZ       PointXYZT;
    typedef pcl::PointXYZRGB    PointXYZRGBT;
    typedef pcl::PointXYZRGBA   PointXYZRGBAT;

    class RealsenseCameraNodelet : public nodelet::Nodelet
    {
        public:
            RealsenseCameraNodelet();
            virtual ~RealsenseCameraNodelet();

        private:
            virtual void onInit();
            void readParameters();
            void findRealsenseVideoDevice();
            void initCameraInfoManagers();

            void initDepthToRGBUVMap();
            int  getUVWithDXY(int depth, int xy, float &uvx, float &uvy);
            void pubRealSensePointsXYZCloudMsg(pcl::PointCloud<pcl::PointXYZ>::Ptr &xyz_input);
            void pubRealSensePointsXYZRGBCloudMsg(pcl::PointCloud<PointType>::Ptr &xyzrgb_input);
            void pubRealSenseDepthImageMsg(cv::Mat& depth_mat);
#ifdef V4L2_PIX_FMT_INZI
            void pubRealSenseInfraredImageMsg(cv::Mat& ir_mat);
#endif
            void pubRealSenseRGBImageMsg(cv::Mat& rgb_mat);
            void initVideoStream();
            int processRGB();
            int processDepth();
            void processRGBD();
            int getNumRGBSubscribers();
            int getNumDepthSubscribers();
            void realsenseConfigCallback(const realsense_camera::realsenseConfig::ConstPtr &config);
            void dynamicReconfigCallback(realsense_camera::RealsenseCameraConfig &config, uint32_t level);
            bool getRGBUV(realsense_camera::get_rgb_uv::Request  &req, realsense_camera::get_rgb_uv::Response &res);
            void timerCallback();

        private:
            bool debug_depth_unit;

            VideoStream rgb_stream;
            VideoStream depth_stream;

            std::string useDeviceSerialNum;

            unsigned char *rgb_frame_buffer;
            unsigned char *depth_frame_buffer;
#ifdef V4L2_PIX_FMT_INZI
            unsigned char *ir_frame_buffer;
#endif

            const int sensor_depth_max;

            std::string realsense_camera_type;

            std::string rgb_frame_id;
            std::string depth_frame_id;

            int rgb_frame_w;
            int rgb_frame_h;

            float depth_unit;
            float depth_scale;

            float depth_fxinv;
            float depth_fyinv;

            float depth_cx;
            float depth_cy;

            int depth_uv_enable_min;
            int depth_uv_enable_max;

            std::string topic_depth_points_id;
            std::string topic_depth_registered_points_id;

            std::string topic_image_rgb_raw_id;
            std::string topic_image_depth_raw_id;
            std::string topic_image_infrared_raw_id;

            //point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr realsense_xyz_cloud;
            pcl::PointCloud<PointType>::Ptr realsense_xyzrgb_cloud;
            bool resize_point_cloud;

            //msgs head
            unsigned int head_sequence_id;
            ros::Time head_time_stamp;

            ros::Publisher realsense_points_pub;
            ros::Publisher realsense_reg_points_pub;

            image_transport::CameraPublisher realsense_rgb_image_pub;
            image_transport::CameraPublisher realsense_depth_image_pub;
#ifdef V4L2_PIX_FMT_INZI
            image_transport::CameraPublisher realsense_infrared_image_pub;
#endif

            // used to read and publish camera calibration parameters
            sensor_msgs::CameraInfoPtr rgb_camera_info;
            sensor_msgs::CameraInfoPtr ir_camera_info;
            std::string rgb_info_url;
            std::string ir_camera_info_url;

            ros::ServiceServer getRGBUVService;

            boost::shared_ptr<dynamic_reconfigure::Server<realsense_camera::RealsenseCameraConfig> > dynamic_reconfigure_server;

            typedef struct
            {
                int depthValue;
                float *uvmap;
            }DepthToRGBUVMap;

            std::map<int, DepthToRGBUVMap> depthToRGBUVMapALL;
            bool isHaveD2RGBUVMap;

            float   center_z;
            int     center_z_count;
            float   center_offset_pixel;

            ros::NodeHandle n;
            ros::NodeHandle private_node_handle_;
            ros::Timer timer;
    };
}

#endif
