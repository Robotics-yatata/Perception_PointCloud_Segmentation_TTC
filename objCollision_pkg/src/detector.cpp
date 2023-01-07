/*#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
//#include "pcl/PCLPointCloud2.h"
//#include "pcl_conversions/pcl_conversions.h"
//#include <sstream>

#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>

/*
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
*/

#include "objCollision.h"





ros::Publisher publisher_closestPointCloud;
ros::Publisher publisher_boundingBox;
ros::Publisher publisher_centroid;






void detectorCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){  


    pcl::PCLPointCloud2 point_cloud;
    pcl_conversions::toPCL(*msg, point_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(point_cloud, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_final(new pcl::PointCloud<pcl::PointXYZ>);




    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.15f, 0.15f, 0.15f);
    vg.filter(*cloud_filtered);



    
    //Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(1);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    int j = 0;
    pcl::PointXYZ center;
    pcl::PointXYZ origin(0, 0, 0);
    float min_dist, dist;
    std_msgs::Float32 Dist;
    min_dist = INFINITY;

    pcl::CentroidPoint<pcl::PointXYZ> closest_centroid;

    for (const auto &cluster : cluster_indices)
    {
        pcl::CentroidPoint<pcl::PointXYZ> centroid;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->push_back((*cloud_filtered)[idx]);
            centroid.add((*cloud_filtered)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //ref: https://pointclouds.org/documentation/classpcl_1_1_centroid_point.html
        centroid.get(center);
        dist = pcl::euclideanDistance(origin, center);
        if (dist < min_dist)
        {
            min_dist = dist;
            *pcl_final = *cloud_cluster;
            closest_centroid = centroid;
        }

        j+=1;
    }

    
    pcl::PointXYZ closest_center;
    closest_centroid.get(closest_center);

    
   

    sensor_msgs::PointCloud2 output;
    pcl_final->header.frame_id = "os_sensor";
    pcl::toROSMsg(*pcl_final, output);
    

    geometry_msgs::Point center_publish;
    center_publish.x = closest_center.x;
    center_publish.y = closest_center.y;
    center_publish.z = closest_center.z;



    //Bounding Box
    //========================================================
    // ref: https://pcl.readthedocs.io/en/latest/moment_of_inertia.html
    // ref: https://stackoverflow.com/questions/49688940/point-cloud-library-rotation-of-axis-alligned-bounding-box



    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(pcl_final);
    feature_extractor.compute();

    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;

    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ position_OBB;

    Eigen::Matrix3f rotational_matrix_OBB;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);

    Eigen::Vector3f point1(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z);
    Eigen::Vector3f point2(min_point_AABB.x, min_point_AABB.y, max_point_AABB.z);
    Eigen::Vector3f point3(max_point_AABB.x, min_point_AABB.y, max_point_AABB.z);
    Eigen::Vector3f point4(max_point_AABB.x, min_point_AABB.y, min_point_AABB.z);
    Eigen::Vector3f point5(min_point_AABB.x, max_point_AABB.y, min_point_AABB.z);
    Eigen::Vector3f point6(min_point_AABB.x, max_point_AABB.y, max_point_AABB.z);
    Eigen::Vector3f point7(max_point_AABB.x, max_point_AABB.y, max_point_AABB.z);
    Eigen::Vector3f point8(max_point_AABB.x, max_point_AABB.y, min_point_AABB.z);

    // ref: http://wiki.ros.org/rviz/DisplayTypes/Marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/os_sensor";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.ns = "bbox";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; 
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    geometry_msgs::Point P_1, P_2, P_3, P_4, P_5, P_6, P_7, P_8;

    P_1.x = point1.x();
    P_1.y = point1.y();
    P_1.z = point1.z();
    P_2.x = point2.x();
    P_2.y = point2.y();
    P_2.z = point2.z();
    P_3.x = point3.x();
    P_3.y = point3.y();
    P_3.z = point3.z();
    P_4.x = point4.x();
    P_4.y = point4.y();
    P_4.z = point4.z();
    P_5.x = point5.x();
    P_5.y = point5.y();
    P_5.z = point5.z();
    P_6.x = point6.x();
    P_6.y = point6.y();
    P_6.z = point6.z();
    P_7.x = point7.x();
    P_7.y = point7.y();
    P_7.z = point7.z();
    P_8.x = point8.x();
    P_8.y = point8.y();
    P_8.z = point8.z();

    marker.points.push_back(P_1);
    marker.points.push_back(P_2);
    marker.points.push_back(P_3);
    marker.points.push_back(P_4);
    marker.points.push_back(P_1);
    marker.points.push_back(P_4);
    marker.points.push_back(P_2);
    marker.points.push_back(P_3);

    marker.points.push_back(P_5);
    marker.points.push_back(P_6);
    marker.points.push_back(P_7);
    marker.points.push_back(P_8);
    marker.points.push_back(P_5);
    marker.points.push_back(P_8);
    marker.points.push_back(P_6);
    marker.points.push_back(P_7);

    marker.points.push_back(P_1);
    marker.points.push_back(P_5);
    marker.points.push_back(P_2);
    marker.points.push_back(P_6);
    marker.points.push_back(P_3);
    marker.points.push_back(P_7);
    marker.points.push_back(P_4);
    marker.points.push_back(P_8);






    publisher_boundingBox.publish(marker);
    publisher_closestPointCloud.publish(output);
    publisher_centroid.publish(center_publish);
    
}






int main(int argc, char **argv){




    ros::init(argc, argv, "detector");
    ros::NodeHandle n;


    publisher_closestPointCloud = n.advertise<sensor_msgs::PointCloud2>("detector/obstacle", 1);
    publisher_boundingBox = n.advertise<visualization_msgs::Marker>("detector/bbox", 1);
    publisher_centroid = n.advertise<geometry_msgs::Point>("detector/center", 1);


    ros::Subscriber subscriber_pointCloud = n.subscribe("/os_cloud_node/points", 1000, detectorCallback);
    


    ros::spin();

}