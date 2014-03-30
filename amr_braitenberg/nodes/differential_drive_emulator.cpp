#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

#include <amr_msgs/WheelSpeeds.h>

ros::Subscriber wheel_speed_subscriber;
ros::Publisher velocity_publisher;
double wheel_diameter;
double distance_between_wheels;

void wheelSpeedCallback(const amr_msgs::WheelSpeeds::ConstPtr& msg)
{
  // Check that the message contains exactly two wheel speeds, otherwise it is
  // meaningless for this emulator.
  if (msg->speeds.size() != 2)
  {
    ROS_WARN("Ignoring WheelSpeeds message because it does not contain two wheel speeds.");
    return;
  }

  geometry_msgs::Twist twist;

  //==================== YOUR CODE HERE ====================
  // Instructions: compute linear and angular components and
  //               fill in the twist message.


  //========================================================

  velocity_publisher.publish(twist);
  ROS_DEBUG("[%.2f %.2f] --> [%.2f %.2f]", msg->speeds[0], msg->speeds[1], twist.linear.x, twist.angular.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "differential_drive_emulator");
  // Read differential drive parameters from server.
  ros::NodeHandle pn("~");
  pn.param("wheel_diameter", wheel_diameter, 0.15);
  pn.param("distance_between_wheels", distance_between_wheels, 0.5);
  // Create subscriber and publisher.
  ros::NodeHandle nh;
  wheel_speed_subscriber = nh.subscribe("/cmd_vel_diff", 100, wheelSpeedCallback);
  velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  // Start infinite loop.
  ROS_INFO("Started differential drive emulator node.");
  ros::spin();
  return 0;
}

