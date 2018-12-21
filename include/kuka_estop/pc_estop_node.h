/*!
 * \pc_estop_node.h
 * \brief Allows for stopping of KUKA youbot.
 *
 * pc_estop_node creates a ROS node that allows a user to stop KUKA youbot. 
 * and sends messages to the /carl_estop topic.
 *
 * \author Diego Leon, UTA - dln.finis@gmail.com
 * \author Russell Toris, WPI - russell.toris@gmail.com
 * \author Chris Dunkers, WPI - cmdunkers@wpi.edu
 * \date November 05, 2018
 */


#ifndef KUKA_ESTOP_TELEOP_H_
#define KUKA_ESTOP_TELEOP_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h>

// the ros subscriber
ros::Publisher estop_pub;

//variables for the parameter values to be stored in
double send_frequency;

/*!
 * Creates and runs the kuka_estop_node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return 0 
 */
int main(int argc, char **argv);

#endif
