/**
 * @file debug_view.h
 * @author Lucas Nogueira (lcasanov@andrew.cmu.edu)
 * @brief Class for holding mapping debug visualization in RVIZ
 * @version 0.1
 * @date 2020-10-07
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <arise_slam_mid360/utils/Twist.h>
#include <arise_slam_mid360/common.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>

namespace arise_slam
{
    class CorrespondenceMarker
    {

        private:
        visualization_msgs::msg::Marker m;
        public:

        CorrespondenceMarker()
        {
            m.id=0;
            m.color.a=1.0;
            m.lifetime = rclcpp::Duration(1, 0);
            m.action = visualization_msgs::msg::Marker::ADD;
            m.type = visualization_msgs::msg::Marker::LINE_LIST;

            m.scale.x = m.scale.y = m.scale.z = 0.01;
            m.color.r = m.color.g = m.color.b = 1.0f;
        }

        void init(std::string ns, std::string frame_id)
        {
            m.ns = ns;
            m.header.frame_id=frame_id;
        }

        void restart()
        {
            m.points.clear();
            m.colors.clear();
        }

        void add(const std::vector<PointType>& fromMap, const PointType fromScan, std_msgs::msg::ColorRGBA color)
        {
            geometry_msgs::msg::Point p;
            p.x = fromScan.x;
            p.y = fromScan.y;
            p.z = fromScan.z;

            for (size_t i = 0; i < fromMap.size(); ++i)
            {
                geometry_msgs::msg::Point p_map;
                PointType p_pcl = fromMap[i];
                p_map.x = p_pcl.x;
                p_map.y = p_pcl.y;
                p_map.z = p_pcl.z;

                m.points.push_back(p);
                m.colors.push_back(color);

                m.points.push_back(p_map);
                m.colors.push_back(color);
            }
        }

        visualization_msgs::msg::Marker getMarkerMsg()
        {
            return m;
        }

        void setTimestamp(double timestamp)
        {
            m.header.stamp = rclcpp::Time(timestamp*1e9);
        }
    };
    class DebugView : rclcpp::Node
    {
    public:
        /**
         * @brief Construct a new Debug View object
         */
        DebugView();

        /**
         * @brief Records a pose obtained in a laser mapping optimization iteration
         * 
         * @param pose 
         * @param timestamp 
         */
        void addOptimizationIterationPose(const Transformd &pose, const double timestamp);

        /**
         * @brief publishes the features obtained from the map
         * 
         * @param corner 
         * @param surf 
         * @param timestamp 
         */
        void publishFeaturesFromMap(const pcl::PointCloud<PointType> &corner,
                                    const pcl::PointCloud<PointType> &surf, const double timestamp);

        /**
         * @brief publishes the features obtained from the current scan frame
         * 
         * @param corner 
         * @param surf 
         * @param timestamp 
         */
        void publishFeaturesFromScan(const pcl::PointCloud<PointType> &corner,
                                    const pcl::PointCloud<PointType> &surf, const double timestamp);

        void addCorrespondences(const std::vector<PointType> &fromMap, const PointType &fromScan, bool corner, Eigen::Vector3d& norm);

        void addObservability(const PointType & fromScan, int feature_type);

        void clearCorrespondences();
        void publishCorrespondences(double timestamp);
        void publishObservibility(double timestamp);
        void publishUncertainty(double uncer_x, double uncer_y, double uncer_z,
                                double uncer_roll, double uncer_pitch, double uncer_yaw);

    private:

        std::string ProjectName;
        // ROS publishers
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_optimization_path_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_features_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_frame_features_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr from_map_corner_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr from_map_surface_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr from_scan_corner_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr from_scan_surface_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr correspondences_pub;;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr from_scan_observibility_pub;;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubUncertaintyX;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubUncertaintyY;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubUncertaintyZ;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubUncertaintyRoll;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubUncertaintyPitch;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubUncertaintyYaw;;

        // ROS messages
        nav_msgs::msg::Path path_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_features_color_pc_, map_features_color_pc_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_features_color_obs_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_surface_from_scan_correspondence, pcl_corner_from_scan_correspondence;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_surface_from_map_correspondence, pcl_corner_from_map_correspondence;

        sensor_msgs::msg::PointCloud2 ros_corner_from_scan_correspondence, ros_surface_from_scan_correspondence;
        sensor_msgs::msg::PointCloud2 ros_corner_from_map_correspondence, ros_surface_from_map_correspondence;
        sensor_msgs::msg::PointCloud2 ros_observibility_from_scan;

        CorrespondenceMarker corner_markers, surface_markers;

        enum Feature_observability : uint8_t {
            rx_cross = 0,               // evaluate for x rotation estimation
            neg_rx_cross = 1,           // evaluate for neg x  rotation estimation
            ry_cross = 2,               // evaluate for y rotation estimation
            neg_ry_cross = 3,           // evaluate for neg y rotation estimation
            rz_cross = 4,               // evaluate for z rotation estimation
            neg_rz_cross = 5,           // evaluate for neg z rotation estimation
            tx_dot = 6,                 // evaluate for x translation
            ty_dot = 7,                 // evaluate for y translation
            tz_dot = 8,                    // evaluate for z translation
            nFeatureObs = 9
        };

    }; // class DebugView

} // namespace arise_slam