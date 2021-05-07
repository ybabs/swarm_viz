#include "dji_odom_adapter/odom_msg_adapter.h"

Dji_odom_adapter::Dji_odom_adapter()
{
    attitude_sub = nh.subscribe("m100/attitude", 10, &Dji_odom_adapter::attitude_callback, this);
    imu_sub = nh.subscribe("m100/imu", 10, &Dji_odom_adapter::imu_callback, this);
    velocity_sub = nh.subscribe("/m100/velocity", 10,  &Dji_odom_adapter::velocity_callback, this);
    position_sub =  nh.subscribe("m100/local_position", 10, &Dji_odom_adapter::local_position_callback, this);
    gps_sub = nh.subscribe("m100/gps_position", 10, &Dji_odom_adapter::gps_callback, this);
    


    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("m100/sdk_control_authority");
    drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("m100/drone_task_control");
    query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("m100/query_drone_version");
    set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("m100/set_local_pos_ref");
    drone_activation_service = nh.serviceClient<dji_sdk::Activation>("m100/activation");

    dji_imu_pub = nh.advertise<sensor_msgs::Imu>("dji_imu", 10, false);
    dji_odom_pub = nh.advertise<nav_msgs::Odometry>("dji_odom", 10, false);
    dji_gps_pub = nh.advertise<sensor_msgs::NavSatFix>("dji_gps", 10, false);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10, false);
    path_pub = nh.advertise<nav_msgs::Path>("m100_path", 10, false);
}


void Dji_odom_adapter::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{

    
    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = "imu_link";

    imu_data.angular_velocity.x =msg->angular_velocity.x;
    imu_data.angular_velocity.y = msg->angular_velocity.y;
    imu_data.angular_velocity.z = msg->angular_velocity.z;

    imu_data.orientation.x = msg->orientation.x;
    imu_data.orientation.y = msg->orientation.y;
    imu_data.orientation.z = msg->orientation.z;
    imu_data.orientation.w = msg->orientation.w;

    imu_data.linear_acceleration.x = msg->linear_acceleration.x;
    imu_data.linear_acceleration.y = msg->linear_acceleration.y;
    imu_data.linear_acceleration.z = msg->linear_acceleration.z;

    //ROS_INFO ("IMU %f", imu_data.linear_acceleration.x);

     dji_imu_pub.publish(imu_data);

    

}

void Dji_odom_adapter::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  gps_position.header.stamp = ros::Time::now();
  gps_position.header.frame_id = "gps_link";
  gps_position.latitude = msg->latitude;
  gps_position.longitude = msg->longitude;
  gps_position.altitude = msg->altitude;


  dji_gps_pub.publish(gps_position);
  

}

void Dji_odom_adapter::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    
    attitude_data.quaternion.w = msg->quaternion.w;
    attitude_data.quaternion.x = msg->quaternion.x;
    attitude_data.quaternion.y = msg->quaternion.y;
    attitude_data.quaternion.z = msg->quaternion.z;
    
 
}

void Dji_odom_adapter::velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg )
{

    velocity_data.vector.x = msg->vector.x;
    velocity_data.vector.y = msg->vector.y;
    velocity_data.vector.z = msg->vector.z;

}

void Dji_odom_adapter::local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    position_data.point.x = msg->point.x;
    position_data.point.y = msg->point.y;
    position_data.point.z = msg->point.z;
}

void Dji_odom_adapter::publish_odom()
{
    odom_data.header.stamp = ros::Time::now();
    odom_data.header.frame_id = "map";
    odom_data.child_frame_id="m100/base_link";
    odom_data.pose.pose.position.x = position_data.point.x;
    odom_data.pose.pose.position.y = position_data.point.y;
    odom_data.pose.pose.position.z = position_data.point.z;
    odom_data.pose.pose.orientation.w = attitude_data.quaternion.w; 
    odom_data.pose.pose.orientation.x = attitude_data.quaternion.x; 
    odom_data.pose.pose.orientation.y = attitude_data.quaternion.y; 
    odom_data.pose.pose.orientation.z = attitude_data.quaternion.z; 
    odom_data.twist.twist.linear.x = velocity_data.vector.x;
    odom_data.twist.twist.linear.y = velocity_data.vector.y;
    odom_data.twist.twist.linear.z = velocity_data.vector.z;
    odom_data.twist.twist.angular.x = imu_data.angular_velocity.x;
    odom_data.twist.twist.angular.y = imu_data.angular_velocity.y;
    odom_data.twist.twist.angular.z = imu_data.angular_velocity.z;

    dji_odom_pub.publish(odom_data);

    // No need to publish the transform here as the robot_localization package will handle it for us..
   publish_transform();
   
}

void Dji_odom_adapter::publish_path()
{

    path_pose.header.frame_id = "m100/base_link";
    path_pose.header.stamp = ros::Time::now();
    uav_path.header.stamp = ros::Time::now();
    uav_path.header.frame_id = "m100/base_link";

    path_pose.pose.position.x = position_data.point.x;
    path_pose.pose.position.y = position_data.point.y;
    path_pose.pose.position.z = position_data.point.z;
    path_pose.pose.orientation.w = attitude_data.quaternion.w;
    path_pose.pose.orientation.x = attitude_data.quaternion.x;
    path_pose.pose.orientation.y = attitude_data.quaternion.y;
    path_pose.pose.orientation.z = attitude_data.quaternion.z;


    uav_path.poses.push_back(path_pose);

    path_pub.publish(uav_path);

}

void Dji_odom_adapter::publish_transform()
{
    int marker_id = 0;
    odom_transform.header.stamp = ros::Time::now();
    odom_transform.header.frame_id="map";
    odom_transform.child_frame_id="m100/base_link";

    //Markers
    marker.header.frame_id = "m100/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "m100_path";
    marker.id = marker_id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    // marker.pose.position.x = position_data.point.x;
    // marker.pose.position.y = position_data.point.y;
    // marker.pose.position.z = position_data.point.z;
    // marker.pose.orientation.w = attitude_data.quaternion.w;
    // marker.pose.orientation.x = attitude_data.quaternion.x;
    // marker.pose.orientation.y = attitude_data.quaternion.y;
    // marker.pose.orientation.z = attitude_data.quaternion.z;
    // marker.scale.x = 0.3;
    // marker.scale.y = 0.3;
    // marker.scale.z = 0.3;
    // marker.color.r = 0.0f;
    // marker.color.g = 1.0f;
    // marker.color.b = 0.0f;
    // marker.color.a = 1.0;
    // marker.lifetime = ros::Duration();

    // marker_pub.publish(marker);

  

    odom_transform.transform.translation.x = position_data.point.x;
    odom_transform.transform.translation.y = position_data.point.y;
    odom_transform.transform.translation.z = position_data.point.z;
    odom_transform.transform.rotation.w = attitude_data.quaternion.w;
    odom_transform.transform.rotation.x = attitude_data.quaternion.x;
    odom_transform.transform.rotation.y = attitude_data.quaternion.y;
    odom_transform.transform.rotation.z = attitude_data.quaternion.z;

     long double recipNorm = 1 / sqrt( odom_transform.transform.rotation.x *  odom_transform.transform.rotation.x 
    +  odom_transform.transform.rotation.y *  odom_transform.transform.rotation.y +  odom_transform.transform.rotation.z 
    *  odom_transform.transform.rotation.z +  odom_transform.transform.rotation.w *  odom_transform.transform.rotation.w);

      odom_transform.transform.rotation.w *= recipNorm;
    odom_transform.transform.rotation.x *= recipNorm;
    odom_transform.transform.rotation.y *= recipNorm;
    odom_transform.transform.rotation.z *= recipNorm;


    odom_broadcaster.sendTransform(odom_transform);

    marker_id++;
   // ROS_INFO ("publishing TF");    

}


void Dji_odom_adapter::obtain_control()
{
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 1;
    sdk_ctrl_authority_service.call(authority);
    if(authority.response.result)
    {
    ROS_INFO("Program has obtained control");
    }

    else
    {
        if(authority.response.ack_data == 3 && authority.response.cmd_set == 1 && authority.response.cmd_id == 0)
        {

            ROS_INFO("Call control Authority again");
        }

        else
        {
            ROS_ERROR("Failed to obtain control authority");
        }
    }
}

void Dji_odom_adapter::drone_activate()
{
   dji_sdk::Activation activation;
   drone_activation_service.call(activation);
   ROS_INFO("Activation successful");

   if(!activation.response.result)
   {
       ROS_ERROR("Drone app not activated. Please check your app key");
   }

   else
   {
       ROS_INFO("Activation Successful");
   }
}

bool Dji_odom_adapter::set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);

  return (bool)localPosReferenceSetter.response.result;
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "dji_odom_publisher");

    Dji_odom_adapter dji_odom_adapter ;

   // dji_odom_adapter.drone_activate();

   // dji_odom_adapter.obtain_control();

   // if(!dji_odom_adapter.set_local_position())
  //  {
    //    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
     //   return 1;
   // }

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        ros::spinOnce();

        // publish odometry at 50Hz
        dji_odom_adapter.publish_odom();
        dji_odom_adapter.publish_path();

        loop_rate.sleep();
    }

    return 0;

}