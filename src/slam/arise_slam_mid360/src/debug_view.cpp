/**
 * @file debug_view.cpp
 * @author Lucas Nogueira (lcasanov@andrew.cmu.edu)
 * @brief 
 * @version 0.1
 * @date 2020-10-07
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <arise_slam_mid360/debug_view.h>

arise_slam::DebugView::DebugView() : Node("debug_view_node")
{
    // Advertise debug topics
    if (!this->get_parameter("PROJECT_NAME", ProjectName)) {
        RCLCPP_INFO(this->get_logger(),  "PROJECT_NAME %s" ,ProjectName.c_str());
    }

    pub_optimization_path_ = this->create_publisher<nav_msgs::msg::Path>(ProjectName+"/lm_optimization_path", 10);
    pub_map_features_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ProjectName+"/map_features", 2);
    pub_frame_features_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ProjectName+"/scan_features", 2);
    from_map_corner_pub =this->create_publisher<sensor_msgs::msg::PointCloud2>(ProjectName+"/corner_from_map", 2);
    from_map_surface_pub=this->create_publisher<sensor_msgs::msg::PointCloud2>(ProjectName+"/surface_from_map", 2);
    from_scan_corner_pub=this->create_publisher<sensor_msgs::msg::PointCloud2>(ProjectName+"/corner_from_scan", 2);
    from_scan_surface_pub=this->create_publisher<sensor_msgs::msg::PointCloud2>(ProjectName+"/surface_from_scan", 2);
    from_scan_observibility_pub=this->create_publisher<sensor_msgs::msg::PointCloud2>(ProjectName+"/observibility_from_scan",2);
    correspondences_pub=this->create_publisher<visualization_msgs::msg::Marker>(ProjectName+"/correspondences", 2);;

    //publish laser odometry uncertainty information
    pubUncertaintyX=this->create_publisher<std_msgs::msg::Float32>(ProjectName+"uncertainty_X", 1);
    pubUncertaintyY=this->create_publisher<std_msgs::msg::Float32>(ProjectName+"uncertainty_Y", 1);
    pubUncertaintyZ=this->create_publisher<std_msgs::msg::Float32>(ProjectName+"uncertainty_Z", 1);
    pubUncertaintyRoll=this->create_publisher<std_msgs::msg::Float32>(ProjectName+"uncertainty_roll", 1);
    pubUncertaintyPitch=this->create_publisher<std_msgs::msg::Float32>(ProjectName+"uncertainty_pitch", 1);
    pubUncertaintyYaw=this->create_publisher<std_msgs::msg::Float32>(ProjectName+"uncertainty_yaw", 1);

    path_.header.frame_id = "sensor_init_rot";

    frame_features_color_pc_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    map_features_color_pc_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    frame_features_color_obs_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl_surface_from_scan_correspondence.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl_corner_from_scan_correspondence.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl_surface_from_map_correspondence.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl_corner_from_map_correspondence.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    corner_markers.init("corners", "sensor_init");
    surface_markers.init("surface", "sensor_init");
}

void arise_slam::DebugView::addOptimizationIterationPose(const Transformd &pose_tf, const double timestamp)
{

    // if(fabs(secs(&path_) - timestamp) > 0.005)
    //     path_.poses.clear();

    path_.header.stamp = rclcpp::Time(timestamp*1e9);

    geometry_msgs::msg::PoseStamped pose;

    pose.pose.orientation.x = pose_tf.rot.y();
    pose.pose.orientation.y = pose_tf.rot.z();
    pose.pose.orientation.z = pose_tf.rot.x();
    pose.pose.orientation.w = pose_tf.rot.w();

    pose.pose.position.x = pose_tf.pos.y();
    pose.pose.position.y = pose_tf.pos.z();
    pose.pose.position.z = pose_tf.pos.x();

    pose.header.stamp = path_.header.stamp;
    pose.header.frame_id = "sensor_init_rot";
    path_.poses.push_back(pose);

    pub_optimization_path_->publish(path_);
}

/**
         * @brief publishes the features obtained from the map.
         * 
         * @param corner 
         * @param surf 
         * @param timestamp 
         */
void arise_slam::DebugView::publishFeaturesFromMap(const pcl::PointCloud<PointType> &corner,
                                                  const pcl::PointCloud<PointType> &surface, const double timestamp)
{
    size_t size_corner = corner.points.size();
    size_t size_surface = surface.points.size();
    map_features_color_pc_->clear();

    for (size_t i = 0; i < size_corner; ++i)
    {
        PointType point = corner.points[i];
        // Corner points are red
        pcl::PointXYZRGB point_rgb;
        point_rgb.r = 255;
        point_rgb.x = point.x;
        point_rgb.y = point.y;
        point_rgb.z = point.z;
        map_features_color_pc_->points.push_back(point_rgb);
    }

    for (size_t i = 0; i < size_surface; ++i)
    {
        PointType point = surface.points[i];
        // Surface points are blue
        pcl::PointXYZRGB point_rgb;
        point_rgb.b = 255;
        point_rgb.x = point.x;
        point_rgb.y = point.y;
        point_rgb.z = point.z;
        map_features_color_pc_->points.push_back(point_rgb);
    }
    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(*map_features_color_pc_, ros_msg);
    ros_msg.header.stamp = rclcpp::Time(timestamp*1e9);
    ros_msg.header.frame_id = "sensor_init";
    pub_map_features_->publish(ros_msg);
}

/**
         * @brief publishes the features obtained from the map.
         * 
         * @param corner 
         * @param surf 
         * @param timestamp 
         */
void arise_slam::DebugView::publishFeaturesFromScan(const pcl::PointCloud<PointType> &corner,
                                                   const pcl::PointCloud<PointType> &surface, const double timestamp)
{
    size_t size_corner = corner.points.size();
    size_t size_surface = surface.points.size();
    frame_features_color_pc_->clear();

    for (size_t i = 0; i < size_corner; ++i)
    {
        PointType point = corner.points[i];
        // Corner points are red
        pcl::PointXYZRGB point_rgb;
        point_rgb.r = 255;
        point_rgb.x = point.x;
        point_rgb.y = point.y;
        point_rgb.z = point.z;
        frame_features_color_pc_->points.push_back(point_rgb);
    }

    for (size_t i = 0; i < size_surface; ++i)
    {
        PointType point = surface.points[i];
        // Surface points are blue
        pcl::PointXYZRGB point_rgb;
        point_rgb.b = 255;
        point_rgb.x = point.x;
        point_rgb.y = point.y;
        point_rgb.z = point.z;
        frame_features_color_pc_->points.push_back(point_rgb);
    }
    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(*frame_features_color_pc_, ros_msg);
    ros_msg.header.stamp = rclcpp::Time(timestamp*1e9);
    ros_msg.header.frame_id = "aft_mapped";
    pub_frame_features_->publish(ros_msg);
}

void copyXYZ(pcl::PointXYZRGB& rgb_point, const PointType &point)
{
    rgb_point.x = point.x;
    rgb_point.y = point.y;
    rgb_point.z = point.z;
}

void setPointColor(pcl::PointXYZRGB& point, const std_msgs::msg::ColorRGBA &color)
{
    point.r = color.r * 255;
    point.g = color.g * 255;
    point.b = color.b * 255;
}

void getRandomColor(std_msgs::msg::ColorRGBA& color)
{
    color.r = (rand() % 256) / 255.0;
    color.g = (rand() % 256) / 255.0;
    color.b = (rand() % 256) / 255.0;
}

void arise_slam::DebugView::addObservability(const PointType & fromScan, int feature_type)
{
    pcl::PointXYZRGB rgb_point;
//    RCLCPP_INFO(this->get_logger(), "feature_type %d", feature_type);
//    RCLCPP_INFO(this->get_logger(), "Feature_observability::rx_cross %d", Feature_observability::rx_cross);
//    RCLCPP_INFO(this->get_logger(), "Feature_observability::ry_cross %d", Feature_observability::ry_cross);
//    RCLCPP_INFO(this->get_logger(), "Feature_observability::rz_cross %d", Feature_observability::rz_cross);
   if(feature_type==Feature_observability::rx_cross || feature_type== Feature_observability::neg_rx_cross)
   {
       pcl::PointXYZRGB point_rgb;
       point_rgb.r = 255;
       point_rgb.x = fromScan.x;
       point_rgb.y = fromScan.y;
       point_rgb.z = fromScan.z;
       rgb_point=point_rgb;
   }


    if(feature_type==Feature_observability::ry_cross|| feature_type==Feature_observability::neg_ry_cross)
    {
        pcl::PointXYZRGB point_rgb;
        point_rgb.g = 255;
        point_rgb.x = fromScan.x;
        point_rgb.y = fromScan.y;
        point_rgb.z = fromScan.z;
        rgb_point=point_rgb;
    }

    if(feature_type==Feature_observability::rz_cross || feature_type==Feature_observability::neg_rz_cross)
    {
        pcl::PointXYZRGB point_rgb;
        point_rgb.b = 255;
        point_rgb.x = fromScan.x;
        point_rgb.y = fromScan.y;
        point_rgb.z = fromScan.z;
        rgb_point=point_rgb;
    }

    frame_features_color_obs_->points.push_back(rgb_point);

}



void arise_slam::DebugView::addCorrespondences(const std::vector<PointType>  &fromMap, const PointType &fromScan, bool corner, Eigen::Vector3d& norm)
{
    std_msgs::msg::ColorRGBA color;
    color.r = (norm(0)+1.0f)/2.0f;
    color.g = (norm(1)+1.0f)/2.0f;
    color.b = (norm(2)+1.0f)/2.0f;
    pcl::PointXYZRGB rgb_point;
    setPointColor(rgb_point, color);
    copyXYZ(rgb_point, fromScan);

    if(corner)
    {
        corner_markers.add(fromMap, fromScan, color);
        pcl_corner_from_scan_correspondence->points.push_back(rgb_point);
    }else
    {
        surface_markers.add(fromMap, fromScan, color);
        pcl_surface_from_scan_correspondence->points.push_back(rgb_point);
    }
    size_t size = fromMap.size();
    for(size_t i = 0; i < size; ++i)
    {
        copyXYZ(rgb_point, fromMap[i]);
        if(corner)
        {
            pcl_corner_from_map_correspondence->points.push_back(rgb_point);
        }else{
            pcl_surface_from_map_correspondence->points.push_back(rgb_point);
        }
    }
}

void arise_slam::DebugView::clearCorrespondences(){
    corner_markers.restart();
    surface_markers.restart();
}

void arise_slam::DebugView::publishCorrespondences(double timestamp)
{
    pcl::toROSMsg(*pcl_corner_from_map_correspondence, ros_corner_from_map_correspondence);
    pcl::toROSMsg(*pcl_surface_from_map_correspondence, ros_surface_from_map_correspondence);
    pcl::toROSMsg(*pcl_corner_from_scan_correspondence, ros_corner_from_scan_correspondence);
    pcl::toROSMsg(*pcl_surface_from_scan_correspondence, ros_surface_from_scan_correspondence);

    ros_corner_from_map_correspondence.header.frame_id = "sensor_init";
    ros_surface_from_map_correspondence.header.frame_id = "sensor_init";
    ros_corner_from_scan_correspondence.header.frame_id = "sensor_init";
    ros_surface_from_scan_correspondence.header.frame_id = "sensor_init";

    ros_corner_from_map_correspondence.header.stamp = rclcpp::Time(timestamp*1e9);
    ros_surface_from_map_correspondence.header.stamp = rclcpp::Time(timestamp*1e9);
    ros_corner_from_scan_correspondence.header.stamp = rclcpp::Time(timestamp*1e9);
    ros_surface_from_scan_correspondence.header.stamp = rclcpp::Time(timestamp*1e9);

    from_map_corner_pub->publish(ros_corner_from_map_correspondence);
    from_map_surface_pub->publish(ros_surface_from_map_correspondence);
    from_scan_corner_pub->publish(ros_corner_from_scan_correspondence);
    from_scan_surface_pub->publish(ros_surface_from_scan_correspondence);

    pcl_corner_from_map_correspondence->clear();
    pcl_surface_from_map_correspondence->clear();
    pcl_corner_from_scan_correspondence->clear();
    pcl_surface_from_scan_correspondence->clear();

    corner_markers.setTimestamp(timestamp);
    surface_markers.setTimestamp(timestamp);

    correspondences_pub->publish(corner_markers.getMarkerMsg());
    correspondences_pub->publish(surface_markers.getMarkerMsg());


}

void arise_slam::DebugView::publishObservibility(double timestamp)
{
    pcl::toROSMsg(*frame_features_color_obs_, ros_observibility_from_scan);
    ros_observibility_from_scan.header.frame_id = "sensor_init";
    ros_observibility_from_scan.header.stamp = rclcpp::Time(timestamp*1e9);
    from_scan_observibility_pub->publish(ros_observibility_from_scan);
    frame_features_color_obs_->clear();
}

void arise_slam::DebugView::publishUncertainty(double uncer_x, double uncer_y, double uncer_z,
                                              double uncer_roll, double uncer_pitch, double uncer_yaw)
{

    std_msgs::msg::Float32 uncertainty_x;
    uncertainty_x.data = uncer_x;
    pubUncertaintyX->publish(uncertainty_x);

    std_msgs::msg::Float32 uncertainty_y;
    uncertainty_y.data = uncer_y;
    pubUncertaintyY->publish(uncertainty_y);

    std_msgs::msg::Float32 uncertainty_z;
    uncertainty_z.data = uncer_z;
    pubUncertaintyZ->publish(uncertainty_z);

    std_msgs::msg::Float32 uncertainty_roll;
    uncertainty_roll.data = uncer_roll;
    pubUncertaintyRoll->publish(uncertainty_roll);

    std_msgs::msg::Float32 uncertainty_pitch;
    uncertainty_pitch.data = uncer_pitch;
    pubUncertaintyPitch->publish(uncertainty_pitch);

    std_msgs::msg::Float32 uncertainty_yaw;
    uncertainty_yaw.data = uncer_yaw;
    pubUncertaintyYaw->publish(uncertainty_yaw);

};

