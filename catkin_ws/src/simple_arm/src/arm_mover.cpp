#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <std_msgs/Float64.h>

ros::Publisher joint1_pub, joint2_pub;

std::vector<float> clamp_at_boundaries(float requested_j1, float requested_j2)
{
  float clamped_j1 = requested_j1;
  float clamped_j2 = requested_j2;

  float min_j1, max_j1, min_j2,max_j2;
  ros::NodeHandle n2;

  std::string node_name = ros::this_node::getName();

  n2.getParam(node_name + "/min_joint_1_angle", min_j1);
  n2.getParam(node_name + "/max_joint_1_angle", max_j1);
  n2.getParam(node_name + "/min_joint_2_angle", min_j2);
  n2.getParam(node_name + "/max_joint_2_angle", max_j2);

  ROS_INFO("Min J1 : %f",min_j1);
  ROS_INFO("Max J1 : %f",max_j1);
  ROS_INFO("Min J2 : %f",min_j2);
  ROS_INFO("Max J2 : %f",max_j2);
  if((requested_j1 < min_j1) || (requested_j1 > max_j1))
  {
    clamped_j1 = std::min(std::max(requested_j1, min_j1),max_j1);
  }
  if((requested_j2 < min_j2) || (requested_j2 > max_j2))
  {
    clamped_j2 = std::min(std::max(requested_j2, min_j2),max_j2);
  }
  std::vector<float> clamped_data = {clamped_j1, clamped_j2};
  return clamped_data;
}

bool handle_safe_move_request(simple_arm::GoToPosition::Request &req, simple_arm::GoToPositionResponse &res)
{
    std::vector<float> joints_angles = clamp_at_boundaries(req.joint_1, req.joint_2);
    // Publish clamped joint angles to the arm
    std_msgs::Float64 joint1_angle, joint2_angle;

    joint1_angle.data = joints_angles[0];
    joint2_angle.data = joints_angles[1];

    joint1_pub.publish(joint1_angle);
    joint2_pub.publish(joint2_angle);

    // Wait 3 seconds for arm to settle
    ros::Duration(3).sleep();

    // Return a response message
    res.msg_feedback = "Joint angles set - j1: " + std::to_string(joints_angles[0]) + " , j2: " + std::to_string(joints_angles[1]);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}


int main(int argc, char** argv)
{
    // Initialize the arm_mover node and create a handle to it
    ros::init(argc, argv, "arm_mover");
    ros::NodeHandle n;

    // Define two publishers to publish std_msgs::Float64 messages on joints respective topics
    joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    // Define a safe_move service with a handle_safe_move_request callback function
    ros::ServiceServer service = n.advertiseService("/arm_mover/safe_move", handle_safe_move_request);
    ROS_INFO("Ready to send joint commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
