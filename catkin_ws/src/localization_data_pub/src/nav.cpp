/*
 * Automatic Addison
 * Website: https://automaticaddison.com
 *   ROS node that converts the user's desired initial pose and goal location
 *   into a usable format.
 * Subscribe:
 *   initialpose : The initial position and orientation of the robot using 
 *                 quaternions. (geometry_msgs/PoseWithCovarianceStamped)
 *   move_base_simple/goal : Goal position and 
 *                           orientation (geometry_msgs::PoseStamped)
 * Publish: This node publishes to the following topics:   
 *   goal_2d : Goal position and orientation (geometry_msgs::PoseStamped)
 *   initial_2d : The initial position and orientation of the robot using 
 *                Euler angles. (geometry_msgs/PoseStamped)
 * From Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *   by Lloyd Brombach
 */
 
// Include statements 
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <move_base_msgs/MoveBaseActionFeedback.h>
 
using namespace std;
 
// Initialize ROS publishers
ros::Publisher pub;
ros::Publisher pub2;
geometry_msgs::PoseStamped currentGoal;
 
// Take move_base_simple/goal as input and publish goal_2d
void send_goal() {
  geometry_msgs::PoseStamped rpyGoal;
  rpyGoal.header.frame_id = "map";
  rpyGoal.header.stamp = ros::Time::now();
  rpyGoal.pose.position.x = -3.529536724090576;
  rpyGoal.pose.position.y = 2.119060516357422;
  rpyGoal.pose.position.z = 0;
  tf::Quaternion q(0, 0, -0.9999999207520988, 0.0003981153051930064);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  q.setRPY(0,0,yaw);
  rpyGoal.pose.orientation.x = q.x();
  rpyGoal.pose.orientation.y = q.y();
  rpyGoal.pose.orientation.z = q.z();
  rpyGoal.pose.orientation.w = q.w();
  currentGoal = rpyGoal;
  pub.publish(rpyGoal);
}
 
// Take initialpose as input and publish initial_2d
void handle_initial_pose() {
  geometry_msgs::PoseWithCovarianceStamped rpyPose;
  rpyPose.header.frame_id = "map";
  rpyPose.header.stamp = ros::Time::now();
  rpyPose.pose.pose.position.x = -5.861403465270996;
  rpyPose.pose.pose.position.y = 8.380659103393555;
  rpyPose.pose.pose.position.z = 0;
  tf::Quaternion q(0, 0, -0.15340019690070383, 0.9881641460763618);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  q.setRPY(0, 0, yaw);
  rpyPose.pose.pose.orientation.x = q.x();
  rpyPose.pose.pose.orientation.y = q.y();
  rpyPose.pose.pose.orientation.z = q.z();
  rpyPose.pose.pose.orientation.w = q.w();
  pub2.publish(rpyPose);
}

void feedbackCallback(const move_base_msgs::MoveBaseActionFeedbackConstPtr& feedback) {
    if (feedback->status.status == feedback->status.SUCCEEDED) {
        ROS_INFO("Robot has reached the goal!");
    } else {
        ROS_INFO("Robot is still moving towards the goal.");
        float currentX = feedback->feedback.base_position.pose.position.x;
        float currentY = feedback->feedback.base_position.pose.position.y;
        float goalX = currentGoal.pose.position.x;
        float goalY = currentGoal.pose.position.y;
        float diffX = abs(currentX - goalX);
        float diffY = abs(currentY - goalY);
        if(diffX <= 1 && diffY <= 1){
          ROS_INFO("Robot has reached the goal!");
          ROS_INFO("YOU CAN SEND ANOTHER GOAL");
          //send next goal
        }
    }
}
 
int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_click_to_2d");
  ros::NodeHandle node;
  pub = node.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 0);
  pub2 = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 0);
  ros::Subscriber feedback_sub = node.subscribe("/move_base/feedback", 10, feedbackCallback);
  //ros::Subscriber sub2 = node.subscribe("initialpose", 0, handle_initial_pose);
  ros::Rate loop_rate(10);
  int initial = 10;
  while (ros::ok()) {
        ros::spinOnce();
        if(initial>0){
          handle_initial_pose();
          initial--;
        }
        else{
          send_goal();
        }
        //handle_initial_pose();
        loop_rate.sleep();
  }
  return 0;
}
