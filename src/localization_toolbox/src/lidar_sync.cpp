#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pub_synced0;
ros::Publisher pub_synced1;
ros::Publisher pub_synced2;
ros::Publisher pub_synced3;


void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msgs0, const sensor_msgs::PointCloud2ConstPtr& cloud_msgs1, const sensor_msgs::PointCloud2ConstPtr& cloud_msgs2, const sensor_msgs::PointCloud2ConstPtr& cloud_msgs3)
{
    // Solve all of perception here...
    //std::cout << "processing callback" << std::endl;
    //std::cout << cloud_msgs0->header << std::endl;
    //std::cout << cloud_msgs1->header << std::endl;
    //std::cout << cloud_msgs2->header << std::endl;
    //std::cout << cloud_msgs3->header << std::endl;
    pub_synced0.publish (cloud_msgs0);
    pub_synced1.publish (cloud_msgs1);
    pub_synced2.publish (cloud_msgs2);
    pub_synced3.publish (cloud_msgs3);


}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "lidar_sync");

    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud 
    //message_filters::Subscriber<PointCloud2> sub_points0(nh, "rslidar_points0", 1);
    //message_filters::Subscriber<PointCloud2> sub_points1(nh, "rslidar_points1", 1);
    //message_filters::Subscriber<PointCloud2> sub_points2(nh, "rslidar_points2", 1);
    //message_filters::Subscriber<PointCloud2> sub_points3(nh, "rslidar_points3", 1);

    message_filters::Subscriber<PointCloud2> sub_points0(nh, "aligned_rslidar_points0", 1);
    message_filters::Subscriber<PointCloud2> sub_points1(nh, "aligned_rslidar_points1", 1);
    message_filters::Subscriber<PointCloud2> sub_points2(nh, "aligned_rslidar_points2", 1);
    message_filters::Subscriber<PointCloud2> sub_points3(nh, "aligned_rslidar_points3", 1);

    //(not working)////////////////
    //TimeSynchronizer<PointCloud2, PointCloud2> sync(sub_points0, sub_points1, 10);
    //sync.registerCallback(boost::bind(&callback, _1, _2));
    ///////////////////////////////

    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2, PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_points0, sub_points1, sub_points2, sub_points3);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    // Create a ROS publisher
    pub_synced0 = nh.advertise<sensor_msgs::PointCloud2> ("synced_rslidar_points0", 1);
    pub_synced1 = nh.advertise<sensor_msgs::PointCloud2> ("synced_rslidar_points1", 1);
    pub_synced2 = nh.advertise<sensor_msgs::PointCloud2> ("synced_rslidar_points2", 1);
    pub_synced3 = nh.advertise<sensor_msgs::PointCloud2> ("synced_rslidar_points3", 1);


    ros::spin();

    return 0;
}