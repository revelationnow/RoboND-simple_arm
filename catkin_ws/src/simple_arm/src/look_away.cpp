#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>

std::vector<double> joints_last_position{0, 0};
bool moving_state = false;
ros::ServiceClient client;

void move_arm_center()
{
  simple_arm::GoToPosition srv;
  srv.request.joint_1 = 1.57;
  srv.request.joint_2 = 1.57;

  if(!client.call(srv))
  {
    ROS_ERROR("Couldn't call");
  }
}

void joint_states_callback(const sensor_msgs::JointState js)
{
  std::vector<double> joints_current_position = js.position;
  double tolerance = 0.0005;
  if (fabs(joints_current_position[0] - joints_last_position[0]) < tolerance &&
      fabs(joints_current_position[1] - joints_last_position[1] < tolerance)
     )
  {
    moving_state = false;
  }
  else
  {
    moving_state = true;
  }
}

void look_away_callback(const sensor_msgs::Image img)
{
  bool uniform_image = true;
  for(int i = 0; i < img.height * img.step; i++)
  {
    if(img.data[i] - img.data[0] != 0) {
      uniform_image = false;
      break;
    }
  }

  if(uniform_image == true && moving_state == false)
  {
    move_arm_center();
  }
}

int main(int argc, char** argv)
{
    // Initialize the look_away node and create a handle to it
    ros::init(argc, argv, "look_away");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from safe_move
    client = n.serviceClient<simple_arm::GoToPosition>("/arm_mover/safe_move");

    // Subscribe to /simple_arm/joint_states topic to read the arm joints position inside the joint_states_callback function
    ros::Subscriber sub1 = n.subscribe("/simple_arm/joint_states", 10, joint_states_callback);

    // Subscribe to rgb_camera/image_raw topic to read the image data inside the look_away_callback function
    ros::Subscriber sub2 = n.subscribe("rgb_camera/image_raw", 10, look_away_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
