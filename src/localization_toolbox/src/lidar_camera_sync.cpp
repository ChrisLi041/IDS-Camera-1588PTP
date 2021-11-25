#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pub_lidar_synced;
ros::Publisher pub_cam_synced;


void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msgs, const sensor_msgs::ImageConstPtr& img_msgs)
{
    pub_lidar_synced.publish (cloud_msgs);
    pub_cam_synced.publish (img_msgs);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "lidar_camera_sync");

    ros::NodeHandle nh;

    message_filters::Subscriber<PointCloud2> sub_points(nh, "/rslidar_points", 1);
    message_filters::Subscriber<Image> sub_image(nh, "/peak_cam_node/image_raw", 1);

    typedef sync_policies::ApproximateTime<PointCloud2, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_points, sub_image);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Create a ROS publisher
    pub_lidar_synced = nh.advertise<sensor_msgs::PointCloud2> ("synced_lidar", 1);
    pub_cam_synced = nh.advertise<sensor_msgs::Image> ("synced_cam", 1);


    ros::spin();

    return 0;
}