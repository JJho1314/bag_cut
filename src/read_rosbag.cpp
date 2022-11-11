#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>

std::string FrontPcd_path;
typedef pcl::PointXYZI PointType;

// rslidar新驱动
struct PointXYZIRT
{
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

void FrontPoint_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    ros::Time time = cloud_msg->header.stamp;
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr fullCloud;
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
    fullCloud.reset(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*cloud_msg, *laserCloudIn);
    double tt = time.toSec();

    int cloudSize = laserCloudIn->points.size();
    for (int i = 0; i < cloudSize; ++i)
    {
        if (!pcl_isfinite(laserCloudIn->points[i].x) ||
            !pcl_isfinite(laserCloudIn->points[i].y) ||
            !pcl_isfinite(laserCloudIn->points[i].z))
            continue;

        if (laserCloudIn->points[i].z < -3. || laserCloudIn->points[i].z > 7.5)
            continue;
        // TODO 针对现在的车设置的
        if (abs(laserCloudIn->points[i].x) < 1.1 && abs(laserCloudIn->points[i].y) < 2.5)
            continue;
        PointType thisPoint;
        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        thisPoint.intensity = laserCloudIn->points[i].intensity;

        fullCloud->push_back(thisPoint);
    }

    // save to pcd file
    // pcl::io::savePCDFileBinary(FrontPcd_path + std::to_string(tt) + ".pcd", *fullCloud);
    pcl::io::savePCDFileASCII(FrontPcd_path + std::to_string(tt) + ".pcd", *fullCloud);

    // pcl::io::savePLYFile(FrontPcd_path + std::to_string(tt) + ".ply", tmp);
    // save to bin file
    // std::ofstream out;
    // std::string save_filename = FrontPcd_path + std::to_string(tt) + ".bin";
    // out.open(save_filename, std::ios::out | std::ios::binary);
    // std::cout << save_filename << " saved" << std::endl;
    // int cloudSize = tmp.points.size();
    // for (int i = 0; i < cloudSize; ++i)
    // {
    //     float point_x = tmp.points[i].x;
    //     float point_y = tmp.points[i].y;
    //     float point_z = tmp.points[i].z;
    //     out.write(reinterpret_cast<const char *>(&point_x), sizeof(float));
    //     out.write(reinterpret_cast<const char *>(&point_y), sizeof(float));
    //     out.write(reinterpret_cast<const char *>(&point_z), sizeof(float));
    // }
    // out.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_rosbag");
    ROS_INFO("Start Read Rosbag and save it as pcd wiht txt");
    ros::NodeHandle nh;

    std::string front_lidar_topic;

    nh.param("front_lidar_topic", front_lidar_topic, std::string("/rslidar_points"));
    nh.param("FrontPcd_path", FrontPcd_path, std::string("/home/jjho/workspace/test_cut/pcd/"));

    ros::Subscriber sub_cloud1 = nh.subscribe(front_lidar_topic, 100, FrontPoint_callback);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    return 0;
}
