#ifndef MSG_ADAPTER
#define MSG_ADAPTER


#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include<nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>


#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/Activation.h>

#include <djiosdk/dji_vehicle.hpp>
#include "dji_sdk/dji_sdk.h"

class TFAdapter
{
    public:
     TFAdapter();
     void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
     void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
     void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
     void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
     void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

     void n3OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
     void a3OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
     


     void publish_odom(); 
     bool set_local_position();
     void publishM100Transform();
     void publishN3Transform();
     void publishA3Transform();

     //void drone_activate();
     //void obtain_control();
    // void publish_path();


    private:
    sensor_msgs::Imu imu_data;
    geometry_msgs::QuaternionStamped attitude_data;
    nav_msgs::Odometry odom_data;
    geometry_msgs::Vector3Stamped velocity_data;
    geometry_msgs::PointStamped position_data;
    geometry_msgs::PoseStamped path_pose;
    sensor_msgs::NavSatFix gps_position;
    visualization_msgs::Marker marker;
    nav_msgs::Path uav_path;
    uint32_t shape = visualization_msgs::Marker::SPHERE;

    geometry_msgs::TransformStamped odom_transform;
    tf::TransformBroadcaster odom_broadcaster;

    geometry_msgs::TransformStamped n3_odom_transform;
    tf::TransformBroadcaster n3_odom_broadcaster;

    geometry_msgs::TransformStamped a3_odom_transform;
    tf::TransformBroadcaster a3_odom_broadcaster;



    nav_msgs::Odometry a3_odom_data;
    nav_msgs::Odometry n3_odom_data;


    ros::Subscriber attitude_sub;
    ros::Subscriber position_sub;
    ros::Subscriber velocity_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber n3_sub;
    ros::Subscriber a3_sub;

    ros::ServiceClient set_local_pos_reference;
    ros::ServiceClient A3_reference;
    ros::ServiceClient N3_reference;
    ros::ServiceClient sdk_ctrl_authority_service;
    ros::ServiceClient drone_task_service;
    ros::ServiceClient query_version_service;
    ros::ServiceClient drone_activation_service;

    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    ros::Publisher dji_odom_pub;
    ros::Publisher dji_imu_pub;
    ros::Publisher dji_gps_pub;
    ros::Publisher path_pub;


};

#endif 