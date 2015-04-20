#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <amr_msgs/MoveToAction.h>
#include <amr_msgs/Obstacle.h>
#include "velocity_controller.h"
#include "diff_velocity_controller.h"
#include "omni_velocity_controller.h"

class MotionControllerNode
{

public:

  /** Node constructor.
    *
    * Creates all required servers, publishers, and listeners. Reads the
    * parameters from server and creates an instance of one of the classes that
    * implement the @ref VelocityController interface, which is later used to
    * compute the values for robot velocity. */
  MotionControllerNode()
  : transform_listener_(ros::Duration(10))
  {
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");
    // Parameters
    pn.param("controller_frequency", controller_frequency_, 10.0);
    pn.param("abort_if_obstacle_detected", abort_if_obstacle_detected_, true);
    // Publishers
    velocity_publisher_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    current_goal_publisher_ = pn.advertise<geometry_msgs::PoseStamped>("current_goal", 0, true); // enable "latching" on a connection
    action_goal_publisher_ = pn.advertise<amr_msgs::MoveToActionGoal>("move_to/goal", 1);
    // Subscribers
    simple_goal_subscriber_ = pn.subscribe<geometry_msgs::PoseStamped>("move_to_simple/goal", 1, boost::bind(&MotionControllerNode::simpleGoalCallback, this, _1));
    if (abort_if_obstacle_detected_)
      obstacles_subscriber_ = nh.subscribe<amr_msgs::Obstacle>("obstacles", 100, boost::bind(&MotionControllerNode::obstaclesCallback, this, _1));
    // Action server
    move_to_server_ = MoveToActionServerUPtr(new MoveToActionServer(pn, "move_to", boost::bind(&MotionControllerNode::moveToCallback, this, _1), false));
    move_to_server_->start();
    // Velocity controller
    createVelocityController();
    ROS_INFO("Started [motion_controller] node.");
  }

  /** This callback is triggered when someone sends an action command to the
    * "move_to" server. */
  void moveToCallback(const amr_msgs::MoveToGoalConstPtr& goal)
  {
    ROS_INFO("Received [move_to] action command.");

    if (!setNewGoal(goal))
      return;

    // Start an infinite loop where in each iteration we try to advance towards
    // the goal and also check if the goal has been preempted (i.e. a new goal
    // was given). The loop is terminated if the goal was reached, or if the
    // node itself shuts down.
    ros::Rate rate(controller_frequency_);
    ros::NodeHandle nh;
    while (nh.ok())
    {
      // Exit if the goal was aborted
      if (!move_to_server_->isActive())
        return;

      // Process pending preemption requests
      if (move_to_server_->isPreemptRequested())
      {
        ROS_INFO("Action preemption requested.");
        if (move_to_server_->isNewGoalAvailable() && setNewGoal(move_to_server_->acceptNewGoal()))
        {
          // A new valid goal was given and accepted. Notify the ActionServer
          // that we preempted and proceed to execute the action.
          //move_to_server_->setPreempted();
        }
        else
        {
          // We have been preempted explicitly, without a new goal therefore we
          // need to shut things down
          publishZeroVelocity();
          // Notify the ActionServer that we have successfully preempted
          move_to_server_->setPreempted();
          // No goal - no actions, just exit the callback
          return;
        }
      }

      // Issue a command to move towards the current goal
      if (!moveTowardsGoal())
      {
        // Finish execution if the goal was reached
        move_to_server_->setSucceeded(amr_msgs::MoveToResult(), "Goal reached.");
        publishZeroVelocity();
        return;
      }

      rate.sleep();
    }

    // We get here only if nh.ok() returned false, i.e. the node has received
    // a shutdown request
    move_to_server_->setAborted(amr_msgs::MoveToResult(), "Aborted. The node has been killed.");
  }

  /** This callback is triggered when someone sends a message with a new target
    * pose to the "move_to_simple/goal" topic.
    *
    * The function simply packs the supplied pose into an action message and
    * re-sends it to the action server for the execution. */
  void simpleGoalCallback(const geometry_msgs::PoseStampedConstPtr& target_pose)
  {
    ROS_INFO("Received target pose through the \"simple goal\" topic. Wrapping it in the action message and forwarding to the server.");
    amr_msgs::MoveToActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *target_pose;
    action_goal_publisher_.publish(action_goal);
  }

  /** This callback is triggered when an obstacle was detected.
    *
    * The current action (if any) will be cancelled. */
  void obstaclesCallback(const amr_msgs::ObstacleConstPtr& obstacle)
  {
    ROS_WARN_THROTTLE(1, "An obstacle was detected. Will stop the robot and cancel the current action.");
    if (move_to_server_->isActive())
      move_to_server_->setAborted(amr_msgs::MoveToResult(), "Aborted. An obstacle was detected.");
    publishZeroVelocity();
  }

private:

  /** Try to advance towards the current target pose.
    *
    * Queries the current position of the base in the odometry frame and
    * passes it to the @ref VelocityController object, which calculates the
    * velocities that are needed to move in the target pose direction.
    *
    * @return false if the goal pose was reached, true if we need to proceed. */
  bool moveTowardsGoal()
  {
    tf::StampedTransform transform;
    try
    {
      ros::Time time;
      std::string str;
      transform_listener_.getLatestCommonTime("odom", "base_footprint", time, &str);
      transform_listener_.lookupTransform("odom", "base_footprint", time, transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("Transform lookup failed (\\odom -> \\base_footprint). Reason: %s.", ex.what());
      return true; // goal was not reached
    }

    Pose current_pose(transform.getOrigin().getX(), transform.getOrigin().getY(), tf::getYaw(transform.getRotation()));
    Velocity velocity = velocity_controller_->computeVelocity(current_pose);

    if (velocity_controller_->isTargetReached())
    {
      ROS_INFO("The goal was reached.");
      return false;
    }
    else
    {
      publishVelocity(velocity);
      return true;
    }
  }

  /** Set new target pose as given in the goal message.
    *
    * Checks if the orientation provided in the target pose is valid.
    * Publishes the goal pose for the visualization purposes.
    *
    * @return true if the goal was accepted. */
  bool setNewGoal(const amr_msgs::MoveToGoalConstPtr& new_goal)
  {
    if (!isQuaternionValid(new_goal->target_pose.pose.orientation))
    {
      ROS_WARN("Aborted. Target pose has invalid quaternion.");
      move_to_server_->setAborted(amr_msgs::MoveToResult(), "Aborted. Target pose has invalid quaternion.");
      return false;
    }
    else
    {
      double x = new_goal->target_pose.pose.position.x;
      double y = new_goal->target_pose.pose.position.y;
      double yaw = tf::getYaw(new_goal->target_pose.pose.orientation);
      Pose pose(x, y, yaw);
      velocity_controller_->setTargetPose(pose);
      poseStampedMsgToTF(new_goal->target_pose, target_pose_);
      target_pose_.frame_id_ = "odom";
      current_goal_publisher_.publish(new_goal->target_pose);
      ROS_INFO_STREAM("New target pose: " << pose);
      return true;
    }
  }

  /** Checks if the quaternion is a valid navigation goal, i.e. has non-zero
    * length and is close to vertical. */
  bool isQuaternionValid(const geometry_msgs::Quaternion& q)
  {
    // Ensure that the quaternion does not have NaN's or infinities
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
    {
      ROS_WARN("Quaternion has NaN's or infinities.");
      return false;
    }

    // Ensure that the length of the quaternion is not close to zero
    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
    if (tf_q.length2() < 1e-6)
    {
      ROS_WARN("Quaternion has length close to zero.");
      return false;
    }

    // Normalize the quaternion and check that it transforms the vertical
    // vector correctly
    tf_q.normalize();
    tf::Vector3 up(0, 0, 1);
    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));
    if (fabs(dot - 1) > 1e-3)
    {
      ROS_WARN("The z-axis of the quaternion is not close to vertical.");
      return false;
    }

    return true;
  }

  /** Publish a velocity command with the given x, y, and yaw values. */
  void publishVelocity(Velocity velocity)
  {
    geometry_msgs::Twist twist = velocity;
    velocity_publisher_.publish(twist);
  }

  /** Publish a velocity command which will stop the robot. */
  inline void publishZeroVelocity()
  {
    publishVelocity(Velocity());
  }

  /** Read the relevant parameters from server and create an instance of
    * the @ref VelocityController class. */
  void createVelocityController()
  {
    ros::NodeHandle pn("~");

    double max_linear_velocity;
    double max_linear_acceleration;
    double linear_tolerance;
    double max_angular_velocity;
    double max_angular_acceleration;
    double angular_tolerance;
    std::string controller;

    pn.param("max_linear_velocity", max_linear_velocity, 0.3);
    pn.param("max_linear_acceleration", max_linear_acceleration, 0.05);
    pn.param("linear_tolerance", linear_tolerance, 0.02);
    pn.param("max_angular_velocity", max_angular_velocity, 0.2);
    pn.param("max_angular_acceleration", max_angular_acceleration, 0.03);
    pn.param("angular_tolerance", angular_tolerance, 0.02);
    pn.param("controller", controller, std::string("omni"));

    if (controller != "diff" && controller != "omni")
    {
      ROS_WARN("Invalid controller parameter (\"%s\"), will create omni.", controller.c_str());
      controller = "omni";
    }

    if (controller == "diff")
    {
      velocity_controller_ = VelocityController::UPtr(
        new DiffVelocityController(
          max_linear_velocity, linear_tolerance,
          max_angular_velocity, angular_tolerance
        )
      );
    }
    else
    {
      //========================= YOUR CODE HERE =========================
      // Instructions: create an instance of OmniVelocityController.
      //
      // Hint: you may copy-paste from the DiffVelocityController case
      //       and adjust the arguments in the call to the constructor to
      //       conform to what you have implemented in that class.


      //==================================================================
    }
  }

  typedef actionlib::SimpleActionServer<amr_msgs::MoveToAction> MoveToActionServer;
  typedef std::unique_ptr<MoveToActionServer> MoveToActionServerUPtr;

  MoveToActionServerUPtr move_to_server_;

  ros::Publisher velocity_publisher_;
  ros::Publisher current_goal_publisher_;
  ros::Publisher action_goal_publisher_;

  ros::Subscriber simple_goal_subscriber_;
  ros::Subscriber obstacles_subscriber_;

  tf::TransformListener transform_listener_;

  VelocityController::UPtr velocity_controller_;

  /// Frequency at which the velocity commands are reissued during action execution.
  double controller_frequency_;

  /// Flag that controls whether the robot should stop and cancel the current action if faced an obstacle.
  bool abort_if_obstacle_detected_;

  /// Current goal pose (in global reference frame).
  tf::Stamped<tf::Pose> target_pose_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_controller");
  MotionControllerNode mcn;
  ros::spin();
  return 0;
}

