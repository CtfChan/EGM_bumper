/*
 * main.hpp
 *
 *  Created on: May 25, 2018
 *      Author: Christopher Chan
 *   Institute: ACFR, ITS group
 */

#pragma once


// ROS headers
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Polygon.h>


// TF headers
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


// Grid map headers
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

// PCL headers
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>


// Ground plane estimation 
#include <pcl/common/angles.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

// PCL_ROS headers
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// Standard Library
#include <iostream>
#include <mutex>
#include <vector>



// Custom Headers
#include <zio_obstacle_msgs/ObstaclesStamped.h>
#include <zio_obstacle_msgs/Obstacle.h>


namespace EGM_ns
{

using Point = pcl::PointXYZ;
using PointColor = pcl::PointXYZRGB;


class EGM_class : public nodelet::Nodelet
{
public:
  /**
   * Constructor
   */
  EGM_class();

  /** 
   * Callback function for lidar points
   */
  void ReceiveLidar(const sensor_msgs::PointCloud2ConstPtr& input);

  /** 
   * Callback function for stereo points
   */
  void ReceiveStereo(const sensor_msgs::PointCloud2ConstPtr& input);

  /** 
   * Callback function for sonar polygons
   */
  void ReceiveSonar(zio_obstacle_msgs::ObstaclesStamped input);


private:

  virtual void onInit();

  /**
   * Uses PCL cropbox to crop point cloud. 
   * @param point_cloud cloud to change (passed by reference)
   * @param max         maximum x, y and z values
   * @param min         minimum x, y and z values
   */
  void CropCloud(pcl::PointCloud<Point>::Ptr& point_cloud, Eigen::Vector4f &max,
                 Eigen::Vector4f &min);

  /**
   * Use PCL VoxelGrid on point_cloud
   * @param[in] point_cloud to voxelize
   */
  void VoxelizeCloud(pcl::PointCloud<Point>::Ptr& point_cloud);


 /**
   * This function will publish the grid map of obstacles as well as the
   * ObstaclesStamped message. We will iterate through the robot-centric
   * grid map and add grid cells that are obstacles into a point cloud.
   * We then apply euclidean clustering to segment it and draw polygons
   * around the obstacles. 
   * @param[in] time recorded from callback function
   * @param[in] offest describes the position of the base_link wrt odom
   */
  void PublishObstacles(ros::Time &time, grid_map::Position &offset, 
                        tf::StampedTransform &base_odom_tf,
                        std::string &sensor);


  // Topics of interest
  std::string lidar_topic_;
  std::string stereo_topic_;
  std::string sonar_topic_;
  std::string fused_grid_topic_;

  // Frames of interest
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  std::string base_footprint_frame_id_;
  std::string lidar_frame_id_;
  std::string stereo_frame_id_;

  // Boolean for publishing 
  bool publish_ground_plane_;
  bool publish_odom_map_;

  // Robot-centric map
  // Note that in this implementation, the odom_map always 
  // follows the robot. The reason why I did this is so 
  // that code can easily be extended to add some sort 
  // of "memory" to the virtual bumper system. 
  grid_map::GridMap odom_map;

  // Store transform for fused grid
  tf::TransformListener listener;

  // Store latest polygon message
  zio_obstacle_msgs::ObstaclesStamped latest_sonar_msg;

  // Subscribers and Publishers, TF listener
  ros::Subscriber lidar_subscriber;
  ros::Subscriber stereo_subscriber;
  ros::Subscriber sonar_subscriber;

  ros::Publisher fused_grid_publisher;    // obstacles grid
  ros::Publisher markers_publisher;     
  ros::Publisher polygons_publisher;

  // Saving realsense and sonar obstacles
  zio_obstacle_msgs::ObstaclesStamped stereo_obs;
  zio_obstacle_msgs::ObstaclesStamped sonar_obs;


  // Counting sonar msgs thus far and number of msgs 
  // with obstacles thus far
  int sonar_count;
  int sonar_obs_count;

  // For thread safety
  std::mutex mu_;

  // Parameters to tune 
  float max_x_, max_y_, max_z_, min_x_, min_y_, min_z_;
  float s_max_x_, s_max_y_, s_max_z_, s_min_x_, s_min_y_, s_min_z_;
  Eigen::Vector4f l_max, l_min;
  Eigen::Vector4f s_max, s_min;

  float leaf_size_;
};


PLUGINLIB_DECLARE_CLASS(EGM_ns, EGM_class, EGM_ns::EGM_class, nodelet::Nodelet);
}


