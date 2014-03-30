#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>

#include <amr_msgs/Ranges.h>
#include <amr_msgs/WheelSpeeds.h>
#include <amr_srvs/SwitchRanger.h>
#include <amr_braitenberg/BraitenbergVehicleConfig.h>
#include "braitenberg_vehicle.h"

ros::Subscriber sonar_subscriber;
ros::Publisher wheel_speeds_publisher;
BraitenbergVehicle::UPtr vehicle;

/** Reconfiguration callback is triggered every time the user changes some
  * field through the rqt_reconfigure interface. */
void reconfigureCallback(amr_braitenberg::BraitenbergVehicleConfig &config, uint32_t level)
{
  //==================== YOUR CODE HERE ====================
  // Instructions: create a new instance of Braitenberg
  //               vehicle with the supplied parameters.
  //
  // Hint: to create an instance of smart pointer use the
  //       following construction:
  //       vehicle = BraitenbergVehicle::UPtr(new BraitenbergVehicle(...))


  // =======================================================

  ROS_INFO("Vehicle reconfigured: type %i, factors %.2f and %.2f", config.type, config.factor1, config.factor2);
}

/** Sonar callback is triggered every time the Stage node publishes new data
  * to the sonar topic. */
void sonarCallback(const amr_msgs::Ranges::ConstPtr& msg)
{
  amr_msgs::WheelSpeeds m;

  //==================== YOUR CODE HERE ====================
  // Instructions: based on the ranges reported by the two
  //               sonars compute the wheel speds and fill
  //               in the WheelSpeeds message.
  //
  // Hint: use vehicle->computeWheelSpeeds(...) function.


  // =======================================================

  wheel_speeds_publisher.publish(m);
  ROS_DEBUG("[%.2f %.2f] --> [%.2f %.2f]", msg->ranges[0].range, msg->ranges[1].range, m.speeds[0], m.speeds[1]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "braitenberg_vehicle");
  ros::NodeHandle nh;
  // Wait until SwitchRanger service (and hence stage node) becomes available.
  ROS_INFO("Waiting for the /switch_ranger service to be advertised...");
  ros::ServiceClient switch_ranger_client = nh.serviceClient<amr_srvs::SwitchRanger>("/switch_ranger");
  switch_ranger_client.waitForExistence();
  // Make sure that the braitenberg sonars are available and enable them.
  amr_srvs::SwitchRanger srv;
  srv.request.name = "sonar_braitenberg";
  srv.request.state = true;
  if (switch_ranger_client.call(srv))
  {
    ROS_INFO("Enabled braitenberg sonars.");
  }
  else
  {
    ROS_ERROR("Braitenberg sonars are not available, shutting down.");
    return 1;
  }
  // Create default vehicle.
  vehicle = BraitenbergVehicle::UPtr(new BraitenbergVehicle);
  // Create subscriber and publisher.
  sonar_subscriber = nh.subscribe("/sonar_braitenberg", 100, sonarCallback);
  wheel_speeds_publisher = nh.advertise<amr_msgs::WheelSpeeds>("/cmd_vel_diff", 100);
  // Create dynamic reconfigure server.
  dynamic_reconfigure::Server<amr_braitenberg::BraitenbergVehicleConfig> server;
  server.setCallback(boost::bind(&reconfigureCallback, _1, _2));
  // Start infinite loop.
  ROS_INFO("Started braitenberg vehicle node.");
  ros::spin();
  return 0;
}

