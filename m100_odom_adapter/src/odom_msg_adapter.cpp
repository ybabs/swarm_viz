#include "TFAdapter/odom_msg_adapter.h"

TFAdapter::TFAdapter()
{
    attitude_sub = nh.subscribe("m100/attitude", 10, &TFAdapter::attitude_callback, this);
    imu_sub = nh.subscribe("m100/imu", 10, &TFAdapter::imu_callback, this);
    velocity_sub = nh.subscribe("/m100/velocity", 10,  &TFAdapter::velocity_callback, this);
    position_sub =  nh.subscribe("m100/local_position", 10, &TFAdapter::local_position_callback, this);
    gps_sub = nh.subscribe("m100/gps_position", 10, &TFAdapter::gps_callback, this);
    n3_sub = nh.subscribe("n3/odometry", 10, &TFAdapter::n3OdometryCallback, this);
    a3_sub = nh.subscribe("/a3/odometry", 10, &TFAdapter::a3OdometryCallback, this);
    


    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("m100/sdk_control_authority");
    drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("m100/drone_task_control");
    query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("m100/query_drone_version");
    set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("m100/set_local_pos_ref");
    A3_reference = nh.serviceClient<dji_sdk::SetLocalPosRef> ("a3/set_local_pos_ref");
    N3_reference = nh.serviceClient<dji_sdk::SetLocalPosRef> ("n3/set_local_pos_ref");
    drone_activation_service = nh.serviceClient<dji_sdk::Activation>("m100/activation");

    // dji_imu_pub = nh.advertise<sensor_msgs::Imu>("dji_imu", 10, false);
    dji_odom_pub = nh.advertise<nav_msgs::Odometry>("dji_odom", 10, false);
    // dji_gps_pub = nh.advertise<sensor_msgs::NavSatFix>("dji_gps", 10, false);
    // marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10, false);
    // path_pub = nh.advertise<nav_msgs::Path>("m100_path", 10, false);

    set_local_position();
}



void TFAdapter::n3OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    n3_odom_data = *msg;
   
    publishN3Transform();
}


void TFAdapter::a3OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  a3_odom_data = *msg;
 // ROS_INFO("Odom Data:  %f", a3_odom_data.twist.twist.angular.x);
  publishA3Transform();
}



void TFAdapter::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
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

void TFAdapter::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  gps_position.header.stamp = ros::Time::now();
  gps_position.header.frame_id = "gps_link";
  gps_position.latitude = msg->latitude;
  gps_position.longitude = msg->longitude;
  gps_position.altitude = msg->altitude;


  dji_gps_pub.publish(gps_position);
  

}

void TFAdapter::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    
    attitude_data.quaternion.w = msg->quaternion.w;
    attitude_data.quaternion.x = msg->quaternion.x;
    attitude_data.quaternion.y = msg->quaternion.y;
    attitude_data.quaternion.z = msg->quaternion.z;
    
 
}

void TFAdapter::velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg )
{

    velocity_data.vector.x = msg->vector.x;
    velocity_data.vector.y = msg->vector.y;
    velocity_data.vector.z = msg->vector.z;

}

void TFAdapter::local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    position_data.point.x = msg->point.x;
    position_data.point.y = msg->point.y;
    position_data.point.z = msg->point.z;
}

void TFAdapter::publish_odom()
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
    publishM100Transform();
   
}


void TFAdapter::publishN3Transform()
{
    n3_odom_transform.header.stamp = ros::Time::now();
    n3_odom_transform.header.frame_id="map";
    n3_odom_transform.child_frame_id="n3/base_link";
 
    n3_odom_transform.transform.translation.x = n3_odom_data.pose.pose.position.x;
    n3_odom_transform.transform.translation.y = n3_odom_data.pose.pose.position.y;
    n3_odom_transform.transform.translation.z = n3_odom_data.pose.pose.position.z;
    n3_odom_transform.transform.rotation.w = n3_odom_data.pose.pose.orientation.w;
    n3_odom_transform.transform.rotation.x = n3_odom_data.pose.pose.orientation.x;
    n3_odom_transform.transform.rotation.y = n3_odom_data.pose.pose.orientation.y;
    n3_odom_transform.transform.rotation.z = n3_odom_data.pose.pose.orientation.z;

     long double recipNorm = 1 / sqrt( n3_odom_transform.transform.rotation.x *  n3_odom_transform.transform.rotation.x 
    +  n3_odom_transform.transform.rotation.y *  n3_odom_transform.transform.rotation.y +  n3_odom_transform.transform.rotation.z 
    *  n3_odom_transform.transform.rotation.z +  n3_odom_transform.transform.rotation.w *  n3_odom_transform.transform.rotation.w);

    n3_odom_transform.transform.rotation.w *= recipNorm;
    n3_odom_transform.transform.rotation.x *= recipNorm;
    n3_odom_transform.transform.rotation.y *= recipNorm;
    n3_odom_transform.transform.rotation.z *= recipNorm;

    n3_odom_broadcaster.sendTransform(a3_odom_transform); 

}

void TFAdapter::publishA3Transform()
{
    a3_odom_transform.header.stamp = ros::Time::now();
    a3_odom_transform.header.frame_id="map";
    a3_odom_transform.child_frame_id="a3/base_link";
 
    a3_odom_transform.transform.translation.x = a3_odom_data.pose.pose.position.x;
    a3_odom_transform.transform.translation.y = a3_odom_data.pose.pose.position.y;
    a3_odom_transform.transform.translation.z = a3_odom_data.pose.pose.position.z;
    a3_odom_transform.transform.rotation.w = a3_odom_data.pose.pose.orientation.w;
    a3_odom_transform.transform.rotation.x = a3_odom_data.pose.pose.orientation.x;
    a3_odom_transform.transform.rotation.y = a3_odom_data.pose.pose.orientation.y;
    a3_odom_transform.transform.rotation.z = a3_odom_data.pose.pose.orientation.z;

     long double recipNorm = 1 / sqrt( a3_odom_transform.transform.rotation.x *  a3_odom_transform.transform.rotation.x 
    +  a3_odom_transform.transform.rotation.y *  a3_odom_transform.transform.rotation.y +  a3_odom_transform.transform.rotation.z 
    *  a3_odom_transform.transform.rotation.z +  a3_odom_transform.transform.rotation.w *  a3_odom_transform.transform.rotation.w);

    a3_odom_transform.transform.rotation.w *= recipNorm;
    a3_odom_transform.transform.rotation.x *= recipNorm;
    a3_odom_transform.transform.rotation.y *= recipNorm;
    a3_odom_transform.transform.rotation.z *= recipNorm;

    a3_odom_broadcaster.sendTransform(a3_odom_transform); 

}

void TFAdapter::publishM100Transform()
{
    int marker_id = 0;
    odom_transform.header.stamp = ros::Time::now();
    odom_transform.header.frame_id="map";
    odom_transform.child_frame_id="m100/base_link";
 

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

}


bool TFAdapter::set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  dji_sdk::SetLocalPosRef A3localPosReferenceSetter;
  dji_sdk::SetLocalPosRef N3localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  A3_reference.call(A3localPosReferenceSetter);
  N3_reference.call(N3localPosReferenceSetter);

  return (bool)localPosReferenceSetter.response.result;
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "dji_odom_publisher");

    TFAdapter tf_adapter ;

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        ros::spinOnce();

        // publish odometry at 50Hz
       // tf_adapter.publish_odom();
        //tf_adapter.publish_path();

        loop_rate.sleep();
    }

    return 0;

}