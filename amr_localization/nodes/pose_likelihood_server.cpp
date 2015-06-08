#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <amr_srvs/GetPoseLikelihood.h>
#include <amr_srvs/GetNearestOccupiedPointOnBeam.h>
#include <amr_srvs/SwitchRanger.h>


class PoseLikelihoodServerNode
{

public:

  //============================== YOUR CODE HERE ==============================
  // Instructions: implemenent the pose likelihood server node including a
  //               constructor which should create all needed servers, clients,
  //               and subscribers, and appropriate callback functions.
  //               GetNearestOccupiedPointOnBeam service allows to query
  //               multiple beams in one service request. Use this feature to
  //               simulate all the laser beams with one service call, otherwise
  //               the time spent on communication with the server will be too
  //               long.
  //
  // Hint: refer to the sources of the previous assignments or to the ROS
  //       tutorials to see examples of how to create servers, clients, and
  //       subscribers.
  //
  // Hint: all the header files that you should (in theory) need are already
  //       included.
  //
  // Hint: in the laser callback it is enough to just store the incoming laser
  //       readings in a class member variable so that they could be accessed
  //       later while processing a service request.
  //
  // Hint: the GetNearestOccupiedPointOnBeam service may return arbitrary large
  //       distance, do not forget to clamp it to [0..max_range] interval.


  //============================================================================

private:

  ros::NodeHandle nh_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_likelihood_server");
  ros::NodeHandle nh;
  // Wait until SwitchRanger service (and hence stage node) becomes available.
  ROS_INFO("Waiting for the /switch_ranger service to be advertised...");
  ros::ServiceClient switch_ranger_client = nh.serviceClient<amr_srvs::SwitchRanger>("/switch_ranger");
  switch_ranger_client.waitForExistence();
  // Make sure that the hokuyo laser is available and enable them.
  amr_srvs::SwitchRanger srv;
  srv.request.name = "scan_front";
  srv.request.state = true;
  if (switch_ranger_client.call(srv))
  {
    ROS_INFO("Enabled hokuyo laser.");
  }
  else
  {
    ROS_ERROR("Hokuyo laser is not available, shutting down.");
    return 1;
  }
  PoseLikelihoodServerNode plsn;
  ros::spin();
  return 0;
}

