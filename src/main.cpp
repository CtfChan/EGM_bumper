#include "main.hpp"

namespace EGM_ns
{

using Point = pcl::PointXYZ;
using PointColor = pcl::PointXYZRGB;


EGM_class::EGM_class() {}

void EGM_class::onInit()
{
    ros::NodeHandle& private_nh = getNodeHandle();
    ros::NodeHandle& public_nh = getNodeHandle();

    NODELET_INFO_STREAM("Reading params: ");
    
    // Topic and frame names
    private_nh.param<std::string>("/EGM_class/lidar_topic", lidar_topic_, "/lidar");
    private_nh.param<std::string>("/EGM_class/stereo_topic", stereo_topic_, "/stereo");
    private_nh.param<std::string>("/EGM_class/sonar_topic", sonar_topic_, "/sonar");
    private_nh.param<std::string>("/EGM_class/fused_grid_topic", fused_grid_topic_, "/fused_grid");

    private_nh.param<std::string>("/EGM_class/odom_frame_id", odom_frame_id_, "/odom");
    private_nh.param<std::string>("/EGM_class/base_link_frame_id", base_link_frame_id_, "/base_link");
    private_nh.param<std::string>("/EGM_class/base_footprint_frame_id", base_footprint_frame_id_, "/base_footprint");
    private_nh.param<std::string>("/EGM_class/lidar_frame_id", lidar_frame_id_, "/lidar_link");
    private_nh.param<std::string>("/EGM_class/stereo_frame_id", stereo_frame_id_, "/stereo_link");

    // Tunable Parameters
    private_nh.param<float>("/EGM_class/leaf_size", leaf_size_, 0.2);

    // Lidar crop box
    private_nh.param<float>("/EGM_class/max_x", max_x_, 10);
    private_nh.param<float>("/EGM_class/max_y", max_y_, 10);
    private_nh.param<float>("/EGM_class/max_z", max_z_, 10);

    private_nh.param<float>("/EGM_class/min_x", min_x_, -10);
    private_nh.param<float>("/EGM_class/min_y", min_y_, -10);
    private_nh.param<float>("/EGM_class/min_z", min_z_, -10);

    l_max << max_x_, max_y_, max_z_, 1.0;
    l_min << min_x_, min_y_, min_z_, 1.0;


    // Realsense crop box
    private_nh.param<float>("/EGM_class/s_max_x", s_max_x_, 10);
    private_nh.param<float>("/EGM_class/s_max_y", s_max_y_, 10);
    private_nh.param<float>("/EGM_class/s_max_z", s_max_z_, 10);

    private_nh.param<float>("/EGM_class/s_min_x", s_min_x_, -10);
    private_nh.param<float>("/EGM_class/s_min_y", s_min_y_, -10);
    private_nh.param<float>("/EGM_class/s_min_z", s_min_z_, -10);

    s_max << s_max_x_, s_max_y_, s_max_z_, 1.0;
    s_min << s_min_x_, s_min_y_, s_min_z_, 1.0;


    // Option to publish
    private_nh.param<bool>("/EGM_class/publish_odom_map", publish_odom_map_, false);


    // Wait until all transforms have been published
    listener.waitForTransform(lidar_frame_id_, base_link_frame_id_,
                              ros::Time(0), ros::Duration(10.0));
    listener.waitForTransform(stereo_frame_id_, base_link_frame_id_,
                              ros::Time(0), ros::Duration(10.0));
    listener.waitForTransform(odom_frame_id_, base_link_frame_id_,
                              ros::Time(0), ros::Duration(10.0));


    // Topic to subscribe to, buffer, and corresponding callback function 
    lidar_subscriber = public_nh.subscribe<sensor_msgs::PointCloud2>
    						(lidar_topic_, 1, &EGM_class::ReceiveLidar, this);

    stereo_subscriber = public_nh.subscribe<sensor_msgs::PointCloud2>
    						(stereo_topic_, 1, &EGM_class::ReceiveStereo, this);

    sonar_subscriber = public_nh.subscribe<zio_obstacle_msgs::ObstaclesStamped>
                            (sonar_topic_, 1, &EGM_class::ReceiveSonar, this);

    fused_grid_publisher = public_nh.advertise<grid_map_msgs::GridMap>
    						("grid_map_fused", 1);

    // Save TF
    tf::StampedTransform prev_transform;
    listener.lookupTransform(odom_frame_id_, base_link_frame_id_, 
        ros::Time(0), prev_transform);

    // Initialize odom_map
    odom_map.add("elevation", 0.0);
    odom_map.add("elevation_low", 0.0);
    odom_map.add("obstacle", 0.0);

    odom_map.setFrameId(odom_frame_id_);
    grid_map::Position offset(prev_transform.getOrigin().x(), 
                                prev_transform.getOrigin().y());
    odom_map.setGeometry(grid_map::Length(30.0, 30.0), leaf_size_, offset);

   	// Topic for visualizing polygons and vb node
    markers_publisher = private_nh.advertise<visualization_msgs::MarkerArray>("obstacle_markers", 1);
    polygons_publisher = private_nh.advertise<zio_obstacle_msgs::ObstaclesStamped>("obstacles", 1);

    // Suppress PCL_ERROR output
    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

}

void EGM_class::ReceiveLidar(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Save time for timing callback
    ros::Time beg = ros::Time::now();

    // Save time for publishing odom_map
    ros::Time time = ros::Time(0);

    // Save important transforms
    tf::StampedTransform lidar_base_tf;
    listener.lookupTransform(base_link_frame_id_, lidar_frame_id_, 
                             ros::Time(0), lidar_base_tf);
    tf::StampedTransform base_odom_tf;
    listener.lookupTransform(odom_frame_id_, base_link_frame_id_, 
                             ros::Time(0), base_odom_tf);

    // Convert to PCL pointcloud 
    pcl::PointCloud <Point>::Ptr point_cloud (new pcl::PointCloud <Point>);
    pcl::fromROSMsg(*input, *point_cloud);

    // Rotate cloud to base_link
    pcl::PointCloud<Point>::Ptr base_link_cloud(new pcl::PointCloud<Point>);
    pcl_ros::transformPointCloud(*point_cloud, *base_link_cloud, lidar_base_tf);

    // Filter the cloud
    EGM_class::CropCloud(base_link_cloud, l_max, l_min);
    EGM_class::VoxelizeCloud(base_link_cloud);

    // Take cloud in odom frame
    pcl::PointCloud<Point>::Ptr odom_cloud(new pcl::PointCloud<Point>);
    pcl_ros::transformPointCloud(*base_link_cloud, *odom_cloud, base_odom_tf);

    // Add mutex for thread safety
    std::unique_lock<std::mutex> locker(mu_);

    // Update map
    grid_map::Position offset(base_odom_tf.getOrigin().x(), 
        base_odom_tf.getOrigin().y());
    odom_map.move(offset);

    
     // Fill the grid map using point cloud
    for (auto &point : odom_cloud->points) 
    {
        grid_map::Position pt_pos(point.x, point.y);
        if (odom_map.isInside(pt_pos))
        {
            // compute distance from point to base_link 
            float dist = std::hypot(point.x - base_odom_tf.getOrigin().x(), 
                                    point.y - base_odom_tf.getOrigin().y());
   
            if (std::isnan(odom_map.atPosition("elevation", pt_pos)))
            {
                odom_map.atPosition("elevation", pt_pos) = point.z; 
            }
            else if ( point.z > odom_map.atPosition("elevation", pt_pos))
            {
                odom_map.atPosition("elevation", pt_pos) = point.z ;
            }

            //update elevation_low
            if (std::isnan(odom_map.atPosition("elevation_low", pt_pos)))
            {
                odom_map.atPosition("elevation_low", pt_pos) = point.z; 
            }
            else if ( point.z < odom_map.atPosition("elevation_low", pt_pos))
            {
                odom_map.atPosition("elevation_low", pt_pos) = point.z ;
            }

        
        }
    }

    // Compute grid cells that are obstacles
    odom_map["obstacle"] += ( ( (odom_map["elevation"].array() - odom_map["elevation_low"].array()) > 0.2) )         
                        .matrix().cast<float>();

    // Publish grid map
    std::string sensor = "lidar";
    EGM_class::PublishObstacles(time, offset, base_odom_tf, sensor);

    // Clear them
    odom_map.clear("elevation");
    odom_map.clear("elevation_low");


    // Print time
    ros::Time end = ros::Time::now();
    std::cerr << "Lidar cb: " << end - beg << std::endl;


}


void EGM_class::ReceiveStereo(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Save time for timing callback
    ros::Time beg = ros::Time::now();

    // Convert to PCL pointcloud 
    pcl::PointCloud <Point>::Ptr point_cloud (new pcl::PointCloud <Point>);
    pcl::fromROSMsg(*input, *point_cloud);

    // Rotate cloud to base_link
    tf::StampedTransform stereo_base_tf;
    listener.lookupTransform(base_link_frame_id_, stereo_frame_id_, 
                             ros::Time(0), stereo_base_tf);
    pcl::PointCloud<Point>::Ptr base_link_cloud(new pcl::PointCloud<Point>);
    pcl_ros::transformPointCloud(*point_cloud, *base_link_cloud, stereo_base_tf);

    // Record time for publishing grid map
    ros::Time time = ros::Time(0);

    // Filter the cloud
    EGM_class::CropCloud(base_link_cloud, s_max, s_min);
    EGM_class::VoxelizeCloud(base_link_cloud);

    // Take cloud in odom frame
    pcl::PointCloud<Point>::Ptr odom_cloud(new pcl::PointCloud<Point>);
    tf::StampedTransform base_odom_tf;
    listener.lookupTransform(odom_frame_id_, base_link_frame_id_, 
                             ros::Time(0), base_odom_tf);
    pcl_ros::transformPointCloud(*base_link_cloud, *odom_cloud, base_odom_tf);

    // Add mutex for thread safety
    std::unique_lock<std::mutex> locker(mu_);

    // Update map
    grid_map::Position offset(base_odom_tf.getOrigin().x(), 
        base_odom_tf.getOrigin().y());
    odom_map.move(offset);

    // Fill the grid map
    for (auto &point : odom_cloud->points) 
    {
        grid_map::Position pt_pos(point.x, point.y);
        if (odom_map.isInside(pt_pos))
        {
            // compute distance from point to base_link 
            float dist = std::hypot(point.x - base_odom_tf.getOrigin().x(), 
                                    point.y - base_odom_tf.getOrigin().y());

            // Inside acceptable range of stereo
            if (dist < 4.0)
            {
                if (std::isnan(odom_map.atPosition("elevation", pt_pos)))
                {
                    odom_map.atPosition("elevation", pt_pos) = point.z; 
                }
                else if ( point.z > odom_map.atPosition("elevation", pt_pos))
                {
                    odom_map.atPosition("elevation", pt_pos) = point.z ;
                }

                //update elevation_low
                if (std::isnan(odom_map.atPosition("elevation_low", pt_pos)))
                {
                    odom_map.atPosition("elevation_low", pt_pos) = point.z; 
                }
                else if ( point.z < odom_map.atPosition("elevation_low", pt_pos))
                {
                    odom_map.atPosition("elevation_low", pt_pos) = point.z ;
                }

                // For close objects (roughly 1m in front), apply lower threshold
                // NOTE: /base_link -> /odom tf does not change along z direction
                // in our car right now so there is no need 
                if (odom_map.atPosition("elevation", pt_pos) > 0.0 
                    && dist < 3.0)
                {
                    std::cerr << "CLOSE OBJECT!!! " << std::endl;
                    odom_map.atPosition("obstacle", pt_pos) = 1.0;
                }
            }
        }
    }


    // Compute obstacles
    odom_map["obstacle"] += ( ( (odom_map["elevation"].array() - odom_map["elevation_low"].array() ) > 0.30 ))    
                        .matrix().cast<float>();

    // Publish obstacles
    std::string sensor = "stereo";
    EGM_class::PublishObstacles(time, offset, base_odom_tf, sensor);

    // Clear them
    odom_map.clear("elevation");
    odom_map.clear("elevation_low");

    ros::Time end = ros::Time::now();
    std::cerr << "Stereo cb: " << end - beg << std::endl;
}


void EGM_class::ReceiveSonar(zio_obstacle_msgs::ObstaclesStamped input)
{
    ++sonar_count;

    if (input.obstacles.size())
    {
        ++sonar_obs_count;
    }

    if (sonar_count >= 3)
    {
        if (sonar_obs_count > 1)
        {
            std::cerr << "Object detected by sonar!!! " << 
                    sonar_obs_count << "/3 were obstacles" << std::endl;
            polygons_publisher.publish(input);
            sonar_obs = input;
        }
        else
        {
            sonar_obs.obstacles.clear();
        }

        // Reset counters
        sonar_obs_count = 0;
        sonar_count = 0;
    }
}


// Private functions


void EGM_class::CropCloud(pcl::PointCloud<Point>::Ptr& point_cloud, Eigen::Vector4f & max,
                 Eigen::Vector4f & min)
{
    pcl::CropBox<Point> boxFilter;
    pcl::PointCloud<Point>::Ptr cropped_cloud(new pcl::PointCloud<Point>);
    boxFilter.setMax(max);
    boxFilter.setMin(min);
    boxFilter.setInputCloud(point_cloud);
    boxFilter.filter(*cropped_cloud);
    point_cloud = cropped_cloud;
}

void EGM_class::VoxelizeCloud(pcl::PointCloud<Point>::Ptr& point_cloud)
{
    pcl::VoxelGrid<Point> vg;
    pcl::PointCloud<Point>::Ptr voxel_cloud(new pcl::PointCloud<Point>);
    vg.setInputCloud(point_cloud);
    vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_); //20cm leaf size
    vg.setSaveLeafLayout(true);
    vg.filter(*voxel_cloud);

    point_cloud = voxel_cloud;
}


void EGM_class::PublishObstacles(ros::Time &time, grid_map::Position &offset, 
                        tf::StampedTransform &base_odom_tf,
                        std::string &sensor)
{
    // Publish grid
    odom_map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;

    // Get submap and compute cells that satisfy condition
    bool isSuccess;
    grid_map::GridMap large_map = odom_map.getSubmap(offset, grid_map::Length(18.0, 18.0), isSuccess);

    // Bumper Points
    pcl::PointCloud<Point>::Ptr bumper_pts(new pcl::PointCloud<Point>);

    // push obstacles into bumper_pts and update large_map 
    for (grid_map::GridMapIterator iterator(large_map);
      !iterator.isPastEnd(); ++iterator) 
    {
        grid_map::Position pos;
        large_map.getPosition(*iterator, pos);
        
        if (large_map.at("obstacle", *iterator) > 0.0)
        {
            Point new_point;
            new_point.x = (float) pos[0];
            new_point.y = (float) pos[1];
            new_point.z = (float) base_odom_tf.getOrigin().z();
            // new_point.z = large_map.at("elevation", *iterator);

            bumper_pts->points.push_back(new_point);

            // update for consistency sake
            large_map.at("obstacle", *iterator) = 1.0;
        }
        
    }

    // Declare msgs
    zio_obstacle_msgs::ObstaclesStamped obstacles_msg;
    visualization_msgs::MarkerArray marker_array_msg;

    // Configure header
    obstacles_msg.header.stamp = ros::Time::now();
    obstacles_msg.header.frame_id = odom_frame_id_;

    // Publishing the clusters (modified version of original code)
    if (bumper_pts->points.size() > 0)
    {
        int polygons = 0;

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
        tree->setInputCloud(bumper_pts);

        // Generate euclidean clusters
        std::vector<pcl::PointIndices> all_clusters;
        pcl::EuclideanClusterExtraction<Point> ec;
        ec.setClusterTolerance(0.4);
        ec.setMinClusterSize(1);
        ec.setMaxClusterSize(2500);
        ec.setSearchMethod(tree);
        ec.setInputCloud(bumper_pts);
        ec.extract(all_clusters);

        // Iterate through each cluster
        for (auto& cluster_indicies : all_clusters)
        {
            // extract a cluster of points
            pcl::PointCloud<Point>::Ptr object_cluster(new pcl::PointCloud<Point>);
            for (auto& point_index : cluster_indicies.indices)
            {
                object_cluster->points.push_back(bumper_pts->points[point_index]);
            }

            pcl::PointCloud<Point>::Ptr cHull_points(new pcl::PointCloud<Point>);
            // generate the convex hull
            if (object_cluster->points.size() >= 4) 
            {
                // Concave hull requires at least 4 points
                pcl::ConcaveHull<Point> cHull;
                cHull.setAlpha(0.50);
                cHull.setInputCloud(object_cluster);
                cHull.reconstruct(*cHull_points);
            } 
            else 
            {
                cHull_points = object_cluster;
            }


            zio_obstacle_msgs::Obstacle polygon_msg;
            visualization_msgs::Marker marker;


            // Three cases to deal with
            // 1. We are able to generate cHull with pts
            // 2. Only one point in cluster
            // 3. Unable to generate cHull so we just push all points into polygon

            if (cHull_points->points.size() > 1) 
            {
                // Push points into polygon and marker msg
                for (auto& hull_point : cHull_points->points)
                {
                  geometry_msgs::Point32 hull_point32_msg;
                  geometry_msgs::Point hull_point_msg;
                  hull_point_msg.x = hull_point32_msg.x = hull_point.x;
                  hull_point_msg.y = hull_point32_msg.y = hull_point.y;
                  hull_point_msg.z = hull_point32_msg.z = hull_point.z;
                  // hull_point_msg.z = hull_point32_msg.z = 1.0;

                  polygon_msg.boundary.points.push_back(hull_point32_msg);
                  marker.points.push_back(hull_point_msg);
                }

                // Complete the polygon
                polygon_msg.boundary.points.push_back(polygon_msg.boundary.points[0]);
                marker.points.push_back(marker.points[0]);

                // Set visualization parameters
                marker.id = polygons++;
                marker.header.frame_id = odom_frame_id_;
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = leaf_size_;
                marker.scale.y = leaf_size_;
                marker.scale.z = 1.0;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.lifetime = ros::Duration(1.0);

                marker_array_msg.markers.push_back(marker);
                obstacles_msg.obstacles.push_back(polygon_msg);
            }


            // In case we have one obstacle, just add another point right on top
            else if (cHull_points->points.size() == 1) 
            {
                // Original point
                for (auto& hull_point : cHull_points->points)
                {
                  geometry_msgs::Point32 hull_point32_msg;
                  geometry_msgs::Point hull_point_msg;
                  hull_point_msg.x = hull_point32_msg.x = hull_point.x;
                  hull_point_msg.y = hull_point32_msg.y = hull_point.y;
                  hull_point_msg.z = hull_point32_msg.z = hull_point.z;
                  // hull_point_msg.z = hull_point32_msg.z = 1.0;

                  polygon_msg.boundary.points.push_back(hull_point32_msg);
                  marker.points.push_back(hull_point_msg);
                }

                // The "made-up" point
                for (auto& hull_point : cHull_points->points)
                {
                  geometry_msgs::Point32 hull_point32_msg;
                  geometry_msgs::Point hull_point_msg;
                  hull_point_msg.x = hull_point32_msg.x = hull_point.x;
                  hull_point_msg.y = hull_point32_msg.y = hull_point.y + 0.1;
                  hull_point_msg.z = hull_point32_msg.z = hull_point.z;
                  // hull_point_msg.z = hull_point32_msg.z = 1.0;

                  polygon_msg.boundary.points.push_back(hull_point32_msg);
                  marker.points.push_back(hull_point_msg);
                }

                polygon_msg.boundary.points.push_back(polygon_msg.boundary.points[0]);
                marker.points.push_back(marker.points[0]);
                marker.id = polygons++;
                marker.header.frame_id = odom_frame_id_;
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = leaf_size_;
                marker.scale.y = leaf_size_;
                marker.scale.z = 1.0;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.lifetime = ros::Duration(1.0);

                marker_array_msg.markers.push_back(marker);
                obstacles_msg.obstacles.push_back(polygon_msg);
            }
        
        
            else if (object_cluster->points.size() > 0 && cHull_points->points.size() <= 0)
            {
                // Push the object cluster points into the polygon array 
                for (auto& hull_point : object_cluster->points)
                {
                  geometry_msgs::Point32 hull_point32_msg;
                  geometry_msgs::Point hull_point_msg;
                  hull_point_msg.x = hull_point32_msg.x = hull_point.x;
                  hull_point_msg.y = hull_point32_msg.y = hull_point.y;
                  hull_point_msg.z = hull_point32_msg.z = hull_point.z;
                  // hull_point_msg.z = hull_point32_msg.z = 1.0;

                  polygon_msg.boundary.points.push_back(hull_point32_msg);
                  marker.points.push_back(hull_point_msg);
                }

                // Complete the polygon
                polygon_msg.boundary.points.push_back(polygon_msg.boundary.points[0]);
                marker.points.push_back(marker.points[0]);

                // Set visualization parameters
                marker.id = polygons++;
                marker.header.frame_id = odom_frame_id_;
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = leaf_size_;
                marker.scale.y = leaf_size_;
                marker.scale.z = 1.0;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.lifetime = ros::Duration(1.0);

                marker_array_msg.markers.push_back(marker);
                obstacles_msg.obstacles.push_back(polygon_msg);
            }


        }         
    }

    // Append sonar to all msgs
    obstacles_msg.obstacles.insert(obstacles_msg.obstacles.end(),
                                   sonar_obs.obstacles.begin(),
                                   sonar_obs.obstacles.end());   

    // Save Realsense obstacles
    if (sensor == "stereo")
    {
        stereo_obs = obstacles_msg;
    }
    // Append stereo obstacles to lidar obstacles. 
    // This ensures that the car doesn't move faster until
    // the obstacle is no longer seen by the Realsense.
    else if (sensor == "lidar")
    {
        obstacles_msg.obstacles.insert(obstacles_msg.obstacles.end(),
                                        stereo_obs.obstacles.begin(),
                                        stereo_obs.obstacles.end());                                 
    } 


    // Publish obstacle markers as well as obstacles
    polygons_publisher.publish(obstacles_msg);
    markers_publisher.publish(marker_array_msg);

    // For visualizing purposes
    if (publish_odom_map_)
    {
         // Publish map
        grid_map::GridMapRosConverter::toMessage(large_map, message);
        fused_grid_publisher.publish(message);
    }
   
    // odom_map.clear("obstacle");
    odom_map["obstacle"] = Eigen::MatrixXf::Zero(odom_map.getSize()(0), odom_map.getSize()(1));
}


// END OF NAMESPACE
}
