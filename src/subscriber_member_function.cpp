#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using std::placeholders::_1;



  
class MinimalSubscriber : public rclcpp::Node {


public:

  visualization_msgs::msg::Marker marker;
  //visualization_msgs::msg::MarkerArray markerArray;
  MinimalSubscriber() : Node("minimal_subscriber") {  
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>("topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    
  
    }

private:
  void topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->point.x);
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->point.y);
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->point.z);
   std::string fromFrameRel = msg->header.frame_id;
   std::string toFrameRel = "map";
   geometry_msgs::msg::TransformStamped transformStamped;
        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
        RCLCPP_INFO(this->get_logger(), "Starting try catch");
        try {
          transformStamped = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
            RCLCPP_INFO(this->get_logger(), "Transform worked");
            geometry_msgs::msg::PointStamped newPoint;
            auto marker = visualization_msgs::msg::Marker();
            newPoint.point.x = 0;
            newPoint.point.y = 0;
            newPoint.point.z = 0;
            tf2::doTransform(*msg, newPoint, transformStamped);
            RCLCPP_INFO(this->get_logger(), "I heard: '%f'", newPoint.point.x);
    	    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", newPoint.point.y);
            RCLCPP_INFO(this->get_logger(), "I heard: '%f'", newPoint.point.z);
            
	    marker.header.frame_id = "map";
	    marker.ns = "basic_shapes";
	    marker.id = 0;
	    marker.type = 1;
	    marker.action = visualization_msgs::msg::Marker::ADD;

	    marker.pose.position.x = newPoint.point.x;
	    marker.pose.position.y = newPoint.point.y;
	    marker.pose.position.z = newPoint.point.z;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;

	    marker.scale.x = 0.4;
	    marker.scale.y = 0.4;
	    marker.scale.z = 0.4;

	    marker.color.r = 0.0f;
	    marker.color.g = 1.0f;
	    marker.color.b = 0.0f;
	    marker.color.a = 1.0;
	    
	    
	    //visualization_msgs::msg::MarkerArray markerArray;
            
	    //markerArray.markers.push_back(marker);
	    
	    //publisher_->publish(markerArray);
            publisher_->publish(marker);
            

        } catch (tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
	}
	RCLCPP_INFO(this->get_logger(), "finished try catch");
	
	
  }
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

