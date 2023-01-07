
#include "objCollision.h"


int collision = 0;

geometry_msgs::Quaternion orientation;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 linear_acceleration;
geometry_msgs::Point position;
geometry_msgs::Vector3 velocity;
geometry_msgs::Point center;
float vel_ang;
double TTC = 0;

ros::Publisher publisher_TTC;
ros::Publisher publisher_TTC_Marker;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  orientation = msg->orientation;
  angular_velocity = msg->angular_velocity;
  linear_acceleration = msg->linear_acceleration;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  position = msg->pose.pose.position;
  velocity = msg->twist.twist.linear;
  vel_ang = atan(velocity.y/velocity.x);
}


void centerCallback(const geometry_msgs::Point &msg) {

  center = msg;
}

void boundingCallback(const visualization_msgs::Marker &marker){


  //p_min = marker.points.at(0); //min min
  //p_max = marker.points.at(10); //max max

  float ang1 = atan(marker.points.at(0).y/marker.points.at(0).x);
  float ang2 = atan(marker.points.at(0).y/marker.points.at(10).x);
  float ang3 = atan(marker.points.at(10).y/marker.points.at(0).x);
  float ang4 = atan(marker.points.at(10).y/marker.points.at(10).x);

  //ang_max = std::max({atan(marker.points.at(0).y/marker.points.at(0).x),atan(marker.points.at(0).y/marker.points.at(10).x),atan(marker.points.at(10).y/marker.points.at(0).x),atan(marker.points.at(10).y/marker.points.at(10).x)});
  //ang_min = std::min({atan(marker.points.at(0).y/marker.points.at(0).x),atan(marker.points.at(0).y/marker.points.at(10).x),atan(marker.points.at(10).y/marker.points.at(0).x),atan(marker.points.at(10).y/marker.points.at(10).x)});
  float ang_max = std::max({ang1,ang2,ang3,ang4});
  float ang_min = std::min({ang1,ang2,ang3,ang4});

  //if (angular_velocity <= std::max({atan(marker.points.at(0).y/marker.points.at(0).x),atan(marker.points.at(0).y/marker.points.at(10).x),atan(marker.points.at(10).y/marker.points.at(0).x),atan(marker.points.at(10).y/marker.points.at(10).x)})
 //&& angular_velocity >= std::min({atan(marker.points.at(0).y/marker.points.at(0).x),atan(marker.points.at(0).y/marker.points.at(10).x),atan(marker.points.at(10).y/marker.points.at(0).x),atan(marker.points.at(10).y/marker.points.at(10).x)})
//) {
  if (vel_ang <= ang_max && vel_ang >= ang_min) {
    TTC = (tf::Vector3(center.x,center.y,center.z)).length() / tf::Vector3(velocity.x,velocity.y,0).length();
  }
  else {
    //ROS_INFO("No collision");
    TTC = 0;
  }

  std::cout << "TTC" << ": " << TTC << std::endl;

}


void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Convert the ROS point cloud message to a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);


    // Convert the quaternion orientation to a 3x3 rotation matrix
    // Get the angular velocity of the vehicle in the inertial frame
    // Estimate the position of the vehicle in the inertial frame
    // Estimate the velocity of the vehicle in the inertial frame
    // Estimate the acceleration of the vehicle in the inertial frame
    
    tf::Matrix3x3 rot(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    tf::Vector3 angular_velocity_inertial = rot * tf::Vector3(angular_velocity.x, angular_velocity.y, angular_velocity.z);
    tf::Vector3 position_inertial(position.x, position.y, position.z);
    tf::Vector3 velocity_inertial(velocity.x, velocity.y, 0);
    tf::Vector3 acceleration_inertial = rot * tf::Vector3(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z) - angular_velocity_inertial.cross(velocity_inertial);


    //double TTC = (obstacle_position_rel.length()) / (obstacle_velocity_rel).length(); 


    //std::cout << "Obstacle Dist" << ": " << obstacle_position_rel.length() << std::endl;
    //std::cout << "Obstacle Vel" << ": " << obstacle_velocity_rel.length() << std::endl;
    





    // Publish the TTC on the "risk_assessor/time_to_collision" topic
    std_msgs::Float32 ttc_msg;
    ttc_msg.data = TTC;
    publisher_TTC.publish(ttc_msg);



    std::vector<double> all_x,all_y,all_z;

    for (int i = 0; i < cloud->points.size(); i++) {
      all_x.push_back(cloud->points[i].x);
      all_y.push_back(cloud->points[i].y);
      all_z.push_back(cloud->points[i].z);
    }

  double min_x = *min_element(all_x.begin(), all_x.end());
  double max_x = *max_element(all_x.begin(), all_x.end());
  double min_y = *min_element(all_y.begin(), all_y.end());
  double max_y = *max_element(all_y.begin(), all_y.end());
  //double min_z = *min_element(all_z.begin(), all_z.end());
  double max_z = *max_element(all_z.begin(), all_z.end());



    // Create a marker message to display the TTC value as text
    visualization_msgs::Marker ttc_marker;
    ttc_marker.header.frame_id = msg->header.frame_id;
    ttc_marker.ns = "os_sensor";
    ttc_marker.id = 0;
    ttc_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    ttc_marker.action = visualization_msgs::Marker::ADD;
    ttc_marker.text = "TTC = " + std::to_string(TTC) + " s";

    //ttc_marker.scale.z = 0.7*(200-TTC)/200.0;  // Set the font size
    ttc_marker.scale.z = 0.7*(50-TTC)/50.0;  // Set the font size
    if (TTC!= 0) {
      ttc_marker.scale.z = 4/TTC;  // Set the font size
    }
    else {
      ttc_marker.scale.z = 0.1;
    }
    ttc_marker.color.r = 0.0;
    ttc_marker.color.g = 1.0;
    ttc_marker.color.b = 0.0;
    ttc_marker.color.a = 1.0;
    ttc_marker.lifetime = ros::Duration();
    ttc_marker.pose.position.x = (max_x-min_x)/2;
    ttc_marker.pose.position.y = (max_y-min_y)/2;
    ttc_marker.pose.position.z = max_z+1;

    // Publish the marker message
    publisher_TTC_Marker.publish(ttc_marker);

}




int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "risk_assessor");
    ros::NodeHandle nh;

    // Subscribe to the "/imu_nav/data" topic
    // Subscribe to the "/gps/rtkfix" topic
    // Subscribe to the "detector/obstacle" topic

    ros::Subscriber subscriber_IMU = nh.subscribe("/imu_nav/data", 1000, imuCallback);
    ros::Subscriber subscriber_Odometry = nh.subscribe("/gps/rtkfix", 1000, odometryCallback);
    ros::Subscriber subscriber_ClosestPointCloud = nh.subscribe("detector/obstacle", 1000, cloudCallback);
    ros::Subscriber subscriber_pointCloud = nh.subscribe("detector/center", 1000, centerCallback);
    ros::Subscriber subscriber_bounding = nh.subscribe("detector/bbox", 1000, boundingCallback);

    
    publisher_TTC = nh.advertise<std_msgs::Float32>("risk_assessor/time_to_collision", 1);
    publisher_TTC_Marker = nh.advertise<visualization_msgs::Marker>("risk_assessor/ttc_marker", 1);

    /*

  // Create a publisher for the "risk_assessor/time_to_collision" topic
  ttc_pub = nh.advertise<std_msgs::Float32>("risk_assessor/time_to_collision", 1000);
  // Create a publisher for the ttc_marker topic
  ttc_marker_pub = nh.advertise<visualization_msgs::Marker>("risk_assessor/ttc_marker", 1);

  */


  ros::spin();

  return 0;
}



