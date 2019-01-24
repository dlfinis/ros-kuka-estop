#include <kuka_estop/kuka_estop_node.h>

void kuka_estop::twistStop() {
    // ROS_INFO("Twist Publish."); 
   // Type of message
  geometry_msgs::Twist stopTwist;

  stopTwist.linear.x = 0;
  stopTwist.linear.y = 0;
  stopTwist.linear.z = 0;
  stopTwist.angular.z = 0;   
  twistPublisher.publish(stopTwist);
}

kuka_estop::kuka_estop() {
  //set the last recieve to now
  last_receive = ros::Time::now();

  //grab the parameters
  ros::NodeHandle private_nh("~");
  private_nh.param<double>("stop_time_delay", stop_time_delay, 1.5);
  private_nh.param<double>("check_frequency", check_frequency, 3.0);

  spoke = false;
  //setup the subscriber
  estop_sub = node.subscribe("kuka_estop", 1000, &kuka_estop::update_time, this);

  //setup to publisher
  twistPublisher = node.advertise<geometry_msgs::Twist>("cmd_vel",1);
  // Connect to the move_base action server
  actionClient = new ActionClient("move_base", true); // create a thread to handle subscriptions.
  last_receive = ros::Time::now();
}

void kuka_estop::estop(void) {
  ros::Time current_time = ros::Time::now();
  //check it has not been too long without a check
  if ((current_time.toSec() - last_receive.toSec()) > stop_time_delay)
  {
    if (!spoke) {
      spoke = true;
      ROS_ERROR("Stopping! Estop Connection Lost.");
      system("espeak \"Stopping! Estop Connection Lost.\"");
    }
    //stop the robot
    twistStop();
    actionClient->waitForServer();
    actionClient->cancelAllGoals();
  }
  else {
    if (spoke)
    {
      ROS_INFO("Estop Connection Resumed.");
      system("espeak \"Estop Connection Resumed.\"");
      spoke = false;
    }
  }
}

void kuka_estop::update_time(const std_msgs::Empty::ConstPtr& msg) {
  //get the current time
  last_receive = ros::Time::now();
}

double kuka_estop::get_frequency() {
  return check_frequency;
}

int main(int argc, char **argv) {
  //initialize the node
  ros::init(argc, argv, "kuka_estop");
  // initialize the estop
  kuka_estop kuka;

  //main loop
  ros::Rate loop_rate(kuka.get_frequency());
  while (ros::ok()) {
    ros::spinOnce();
    kuka.estop();
    loop_rate.sleep();
  }
  return EXIT_SUCCESS;
}
