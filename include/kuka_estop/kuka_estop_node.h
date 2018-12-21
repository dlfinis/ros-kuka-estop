/*!
 * \kuka_estop_node.h
 * Allows the stopping of the segway base without losing power to peripherals
 *
 * kuka_estop_node creates a ROS node that allows the stopping of the robot after a specified amount of time.
 * This node listens to a /kuka_estop topic
 * and sends messages to the /move_base actionlib to stop the current execution of a goal.
 *
 * \author Diego Leon, UTA - dln.finis@gmail.com
 * \author Russell Toris, WPI - russell.toris@gmail.com
 * \author Chris Dunkers, WPI - cmdunkers@wpi.edu
 * \date November 05, 2018
 */

#ifndef KUKA_ESTOP_H_
#define KUKA_ESTOP_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ActionClient;

class kuka_estop
{
public:
  kuka_estop();
  double get_frequency();

  /*!
   * estop function to check the time difference to see if any goal position in move_base should be cancelled
   *
   * \param msg the empty message
   */
  void estop(void);
  void twistStop(void);

private:
  //main node handle
  ros::NodeHandle node;

  // the ros subscriber
  ros::Subscriber estop_sub;

  // ros publisher to cmd_vel
  ros::Publisher twistPublisher;

  //variables for the parameter values to be stored in
  double stop_time_delay;
  double check_frequency;
  ros::Time last_receive;



  //variable used to make sure robot only says the connection lost once
  bool spoke;

  // A handle for the move_base action client thread
  ActionClient* actionClient;


  /*!
   * kuka_estop topic callback function.
   *
   * \param msg the empty message
   */
  void update_time(const std_msgs::Empty::ConstPtr& msg);

};

/*!
 * Creates and runs the kuka_estop_node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS
 */
int main(int argc, char **argv);

#endif
