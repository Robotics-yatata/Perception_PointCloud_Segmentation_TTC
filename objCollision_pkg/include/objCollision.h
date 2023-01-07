#ifndef OBJCOLLISION_H
#define OBJCOLLISION_H

#include <iostream>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <ros/ros.h> //ROS msg that will be published to vtracker
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <std_msgs/Float32.h>
#include <pcl/common/distances.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#define foreach BOOST_FOREACH
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <visualization_msgs/Marker.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Float32.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include "tf/transform_datatypes.h"


#endif