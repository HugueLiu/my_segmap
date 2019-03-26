#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <typeinfo>



int count;
bool save;

void get_target_call(const sensor_msgs::PointCloud2::ConstPtr& msg){
    ++count;
    ROS_INFO("%d, %dx%d", count, msg->width, msg->height);

    if(save){
        save = false;
        ROS_INFO("saving");
        pcl::PCLPointCloud2 pcl_point_cloud_2;
        ROS_INFO("1");
        pcl::PointCloud<pcl::PointXYZI> PointICloud;
        ROS_INFO("2");
        pcl_conversions::toPCL(*msg, pcl_point_cloud_2);
        ROS_INFO("3");
        ROS_INFO("%d", pcl_point_cloud_2.height);
        pcl::fromPCLPointCloud2(pcl_point_cloud_2, PointICloud);
        ROS_INFO("4");
        pcl::io::savePCDFileASCII("/home/liu/.segmap/target.pcd", PointICloud);
        ROS_INFO("finished");
    }
}

void get_control_call(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("received: %s", msg->data.c_str());

    if(msg->data == "save"){
        save = true;
    }
}

int main(int argc, char **argv)
{
    count = 0;
    save = false;
    ros::init(argc, argv, "target_rep_sub");

    ros::NodeHandle nh("segmapper");
    ROS_INFO(nh.getNamespace().c_str());
    pcl:
    ros::Subscriber sub = nh.subscribe("/segmatch/target_representation", 20, get_target_call);
    ros::Subscriber sub_msg = nh.subscribe("/segmatch/control", 10, get_control_call);

    ROS_INFO("Ready to get point clouds.");
    ros::spin();

    return 0;
}