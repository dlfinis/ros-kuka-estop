/*!
 * \pc_estop_node.cpp
 * \brief Allows for stopping of KUKA.
 *
 * pc_estop_node creates a ROS node that allows a user to stop KUKA.
 * and sends messages to the /KUKA_estop topic.
 *
 * \author Diego Leon, UTA - dln.finis@gmail.com
 * \author Russell Toris, WPI - russell.toris@gmail.com
 * \author Chris Dunkers, WPI - cmdunkers@wpi.edu
 * \date November 05, 2018
 */

#include <ros/ros.h>
#include <std_msgs/Empty.h>

/*!
 * Creates and runs the kuka_estop_node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS
 */
int main(int argc, char **argv)
{
  // initialize the node
  ros::init(argc, argv, "pc_estop");

  // main node handle
  ros::NodeHandle node;

  // grab the parameters
  ros::NodeHandle private_nh("~");
  double send_frequency;
  private_nh.param<double>("send_frequency", send_frequency, 3.0);

  //setup the publisher
  ros::Publisher estop_pub = node.advertise<std_msgs::Empty>("kuka_estop", 1);
  std_msgs::Empty msg;

  //main publish loop
  ros::Rate loop_rate(send_frequency);
  while (ros::ok())
  {
    estop_pub.publish(msg);
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
