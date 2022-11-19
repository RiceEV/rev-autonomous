#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudSubcriber {
public:
    PointCloudSubcriber (ros::NodeHandle &nh, std::string topic_name, size_t buffer):
        nh_(nh), cloud_(new pcl::PointCloud<pcl::PointXYZI>()) {
        sub_ = nh_.subscribe(topic_name, buffer, &PointCloudSubcriber::Callback, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud_viz", 100);
    }


private:

    void Callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_ptr) {
        pcl::fromROSMsg(*cloud_ptr, *cloud_);
        std::cout << cloud_->size() << std::endl;
        
        sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_, *cloud_ptr_output);
        cloud_ptr_output->header.stamp = cloud_ptr->header.stamp;
        cloud_ptr_output->header.frame_id = "map";

        pub_.publish(cloud_ptr_output);
    }

   ros::NodeHandle nh_;
   ros::Subscriber sub_;
   ros::Publisher pub_;
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_subscriber");
    ros::NodeHandle nh;

    PointCloudSubcriber sub (nh, "/carla/ego_vehicle/lidar", 100);

    ros::spin();
}