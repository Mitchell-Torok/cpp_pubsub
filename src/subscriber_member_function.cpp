#include <chrono>
#include <memory>
#include <cstdlib>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "qr_custom_message/msg/qr_point_stamped.hpp"
#include "comp3431_interfaces/srv/map_info.hpp"
#include "comp3431_interfaces/msg/qr_code_block.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using std::placeholders::_1;
using namespace std::chrono_literals;


  
class MinimalSubscriber : public rclcpp::Node {


public:

  visualization_msgs::msg::Marker marker;
  //visualization_msgs::msg::MarkerArray markerArray;
  MinimalSubscriber() : Node("minimal_subscriber") {  
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    subscription_ = this->create_subscription<qr_custom_message::msg::QrPointStamped>("topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    
    
    client = this->create_client<comp3431_interfaces::srv::MapInfo>("set_map_info");
  
    cmd_subscription_ = this->create_subscription<std_msgs::msg::String>("cmd", 10, std::bind(&MinimalSubscriber::command_recieved, this, _1));
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    
    x = 0;
    }

private:
  int x; //Variable for marker id so no to markers have the same ID
  //std::vector<comp3431_interfaces::srv::MapInfo::Request> request;
  std::vector<comp3431_interfaces::msg::QRCodeBlock> blocks;
  void command_recieved(const std_msgs::msg::String::SharedPtr msg) {
  	
  	//TODO CLIENT THINGY TO SEND DATA 
  	std::string command = msg->data.c_str();
  	if (command.compare("stop") == 0) {
  		RCLCPP_INFO(this->get_logger(), "stop");
  		

		  while (!client->wait_for_service(1s)) {
		    if (!rclcpp::ok()) {
		      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
		      break;
		    }
		    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
		  }
		  auto request = std::make_shared<comp3431_interfaces::srv::MapInfo::Request>();
		  request->blocks = blocks;
		  auto result = client->async_send_request(request);
  		
  		
  	}
  	
  	RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  
  }
  void topic_callback(const qr_custom_message::msg::QrPointStamped::SharedPtr msg) {
    std::string fromFrameRel = msg->header.frame_id;
    std::string toFrameRel = "map";
    geometry_msgs::msg::TransformStamped transformStamped;

        RCLCPP_INFO(this->get_logger(), "Starting try catch");
        try {
          transformStamped = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel,tf2::TimePointZero);
            RCLCPP_INFO(this->get_logger(), "Transform worked");
            
            
                 auto block = comp3431_interfaces::msg::QRCodeBlock();
		  block.text =  msg->data;
		  block.pose.position.x = msg->point.x;
		  block.pose.position.y = msg->point.y;
		  block.pose.position.z = msg->point.z;
		  
		blocks.push_back(block);
            
            geometry_msgs::msg::PointStamped newPoint;
            geometry_msgs::msg::PointStamped beforePoint;
           
            beforePoint.header = msg->header;
            beforePoint.point  = msg->point;
            
            tf2::doTransform(beforePoint, newPoint, transformStamped);
            
            
            //PUBLISH THE MARKER
	    auto marker = visualization_msgs::msg::Marker();
            marker.id = x;
            x++;
	    marker.header.frame_id = "map";
	    marker.ns = "basic_shapes";
	    marker.text = msg->data;
	    
	    marker.action = visualization_msgs::msg::Marker::ADD;

	    marker.pose.position.x = newPoint.point.x;
	    marker.pose.position.y = newPoint.point.y;
	    marker.pose.position.z = newPoint.point.z;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;

	    marker.scale.x = 0.2;
	    marker.scale.y = 0.2;
	    marker.scale.z = 0.2;

	    marker.color.a = 1.0;
	    
	    if (msg->data.compare("apple") == 0) {
            	marker.color.r = 1.0f;
	    	marker.color.g = 0.0f;
	    	marker.color.b = 0.0f;
	    	marker.type = 2;
            } else if (msg->data.compare("car") == 0) {
            	marker.color.r = 0.5f;
	    	marker.color.g = 0.5f;
	    	marker.color.b = 0.5f;
	    	marker.type = 1;
            } else if (msg->data.compare("orange") == 0) {
            	marker.color.r = 1.0f;
	    	marker.color.g = 0.65f;
	    	marker.color.b = 0.0f;
	    	marker.type = 2;
            } else if (msg->data.compare("banana") == 0) {
            	marker.color.r = 1.0f;
	    	marker.color.g = 1.0f;
	    	marker.color.b = 0.0f;
	    	marker.type = 3;
            } else {
            	marker.color.r = 0.0f;
	    	marker.color.g = 1.0f;
	    	marker.color.b = 0.0f;
	    	marker.type = 4;
            }
	    
            publisher_->publish(marker);
            
            
            //PUBLISH THE TEXT ABOVE THE IMAGE
            auto markerText = visualization_msgs::msg::Marker();
            markerText.id = x;
            x++;
	    markerText.header.frame_id = "map";
	    markerText.ns = "basic_shapes";
	    
	    markerText.action = visualization_msgs::msg::Marker::ADD;

	    markerText.pose.position.x = newPoint.point.x;
	    markerText.pose.position.y = newPoint.point.y;
	    markerText.pose.position.z = newPoint.point.z + 0.4; 

	    markerText.scale.z = 0.3;

	    markerText.color.a = 1.0;
	    markerText.type = 9;

            markerText.text = msg->data;
	    
            publisher_->publish(markerText);
            
        
            

        } catch (tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
	}
	RCLCPP_INFO(this->get_logger(), "finished try catch");
	
	
  }
  rclcpp::Subscription<qr_custom_message::msg::QrPointStamped>::SharedPtr subscription_;
  rclcpp::Client<comp3431_interfaces::srv::MapInfo>::SharedPtr client;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_subscription_;
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

