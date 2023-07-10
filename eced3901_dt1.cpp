/*
Code for DT1
Adapted from: V. Sieben
Written by: Iain Duncan, Denis Hadzimurtezic, Joe Anderson and Yong You - GROUP 14 ECED3901
Version 1.0
Date: Feb 4, 2023
License: GNU GPLv3
*/

// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono>		// Timer functions
#include <functional>		// Arithmetic, comparisons, and logical operations
#include <memory>		// Dynamic memory management
#include <string>		// String functions
#include <cmath>

// ROS Client Library for C++
#include "rclcpp/rclcpp.hpp"
 
// Message types
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;


// Create the node class named SquareRoutine
// It inherits rclcpp::Node class attributes and functions
class SquareRoutine : public rclcpp::Node
{
  public:
	// Constructor creates a node named Square_Routine. 
	SquareRoutine() : Node("Square_Routine")
	{
		// Create the subscription
		// The callback function executes whenever data is published to the 'topic' topic.
		subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&SquareRoutine::topic_callback, this, _1));
          
		// Create the publisher
		// Publisher to a topic named "topic". The size of the queue is 10 messages.
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
      
	  	// Create the timer
	  	timer_ = this->create_wall_timer(10ms, std::bind(&SquareRoutine::timer_callback, this)); 	  
	}

  private:
	void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		x_now = msg->pose.pose.position.x;
		y_now = msg->pose.pose.position.y;
		//w_now = msg->pose.pose.position.w;
		
		//get current quaternion values, x and y redundant but easier to follow this way
		qua_x = msg->pose.pose.orientation.x;
		qua_w = msg->pose.pose.orientation.w;
		qua_z = msg->pose.pose.orientation.z;
		qua_y = msg->pose.pose.orientation.y;
	}
	
	void timer_callback()
	{
		geometry_msgs::msg::Twist msg;
        	
		// Calculate distance travelled from initial
		d_now =	pow( pow(x_now - x_init, 2) + pow(y_now - y_init, 2), 0.5 );
		
		qua_normalize = pow( (pow(qua_x,2) + pow(qua_y,2) + pow(qua_z,2) + pow(qua_w,2)) ,0.5); 
		//RCLCPP_INFO(this->get_logger(), "normalized value: %f", qua_normalize);// print thee normalized value
		
		//normalizing the quaternion values 
		qua_x = qua_x/qua_normalize; 
		qua_y = qua_y/qua_normalize; 
		qua_z = qua_z/qua_normalize; 
		qua_w = qua_w/qua_normalize; 
		
		
		
		/*
		conversion from quaternion to euler 
		Retrieved from: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
		*/
		
		siny_cosp = 2 * (qua_w *qua_z+qua_x*qua_y);
		cosy_cosp = 1 - 2*(qua_z * qua_z+qua_y*qua_y); 
		angular_now = ((180/M_PI) * std::atan2(siny_cosp, cosy_cosp)); //converts from radians to degrees
		
	
		RCLCPP_INFO(this->get_logger(), "current angle: %f", angular_now); //print current angle
		
		// Keep moving if not reached last distance target
		if (d_now < d_aim)
		{
			msg.linear.x = x_vel; 
			msg.angular.z = 0;
			publisher_->publish(msg);
					
		}
		
		//turn left until degree turned is essentially zero
		 
		else if (abs(angle_wrap(angular_now - angular_init)  < angular_aim)){
			RCLCPP_INFO(this->get_logger(), "TURNING. Angle turned: %f", abs(angle_wrap(angular_now - angular_init))); //debugging
			msg.linear.x = 0;
			msg.angular.z = 0.1; //angular speed	
			publisher_->publish(msg); 
			
		}
		// If done step, stop
		else
		{
			msg.linear.x = 0; 
			msg.angular.z = 0; 
			publisher_->publish(msg); 
			
			last_state_complete = 1;
		}	

		sequence_statemachine();
		//RCLCPP_INFO(this->get_logger(), "Published cmd_vel.");
	}
	
	void sequence_statemachine()
	{
		if (last_state_complete == 1)
		{
			switch(count_) 
			{
			  case 0:
			    move_distance(1.0);
			    curr_angle = angular_now;
			    RCLCPP_INFO(this->get_logger(), "case 0");//print current case
			    break;
			  case 1:
			    turn_angular(90); //first 90 degree turn
			    RCLCPP_INFO(this->get_logger(), "case 1");//print current case
			    break;
			  case 2:
			    move_distance(1.0);
			    curr_angle = angular_now;
			    RCLCPP_INFO(this->get_logger(), "case 2");//print current case
			    break;
			  case 3:
			    turn_angular(90); //second 90 degree turn 
			    RCLCPP_INFO(this->get_logger(), "case 3");//print current case
			    break;
			  case 4:
			    move_distance(1.0);
			    curr_angle = angular_now;
			    RCLCPP_INFO(this->get_logger(), "case 4");//print current case
			    break; 
			  case 5:
			    turn_angular(90); //third 90 deg turn 
			    RCLCPP_INFO(this->get_logger(), "case 5");//print current case
			    break;
			  case 6:
			    move_distance(1.0);
			    curr_angle = angular_now;
			    RCLCPP_INFO(this->get_logger(), "case 6");//print current case
			    break;
			  case 7:
			    turn_angular(90); // fourth 90 deg turn
			    RCLCPP_INFO(this->get_logger(), "case 7");//print current case
			    break;
			  default:
			    break;
			}
		}			
	}
	
	// Set the initial position as where robot is now and put new d_aim in place	
	void move_distance(double distance)
	{
		d_aim = distance;
		x_init = x_now;
		y_init = y_now;		
		
		count_++;		// advance state counter
		last_state_complete = 0;	
	}
	
	void turn_angular(double angle)
	{
		angular_aim = angle;
		angular_init = angular_now;
		count_++; 
		last_state_complete = 0; 
	}	
	
	
	double angle_wrap(double angle){
	
		 angle = fmod(angle+180,360); 
		 
		 if(angle < 0){
		 	angle += 360;
		 } 
		 
		 return angle-180;
	}
	

	// Declaration of subscription_ attribute
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
         
	// Declaration of publisher_ attribute      
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	
	// Declaration of the timer_ attribute
	rclcpp::TimerBase::SharedPtr timer_;
	
	// Declaration of Class Variables
	double x_vel = 0.1;
	double x_now = 0, x_init = 0, y_now = 0, y_init = 0;
	double current_angle = 0; 
	double d_now = 0, d_aim = 0;
	double qua_w=0,qua_z=0, qua_x=0, qua_y=0;
    	double siny_cosp=0,cosy_cosp=0,angular_now=0;
    	double angular_aim=0;
    	double angular_init = 0; 
	size_t count_ = 0;
	double qua_normalize = 0; 
	int last_state_complete = 1;
	double curr_angle = 0; 
};
    	


//------------------------------------------------------------------------------------
// Main code execution
int main(int argc, char * argv[])
{
	// Initialize ROS2
	rclcpp::init(argc, argv);
  
	// Start node and callbacks
	rclcpp::spin(std::make_shared<SquareRoutine>());
 	//it works on github
	// Stop node 
	rclcpp::shutdown();
	return 0;
}
