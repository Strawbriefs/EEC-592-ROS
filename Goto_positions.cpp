#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
float x[4],y[4];

int main(int argc, char** argv) {
	ros::init(argc, argv, "simple_navigation_goals");

	MoveBaseClient ac("move_base", true);

	 while(!ac.waitForServer(ros::Duration(5.0))) {
	 	ROS_INFO("Waiting for the move_base action server to come up");
	 }

 	move_base_msgs::MoveBaseGoal goal;

 	goal.target_pose.header.frame_id = "map";
  	goal.target_pose.header.stamp = ros::Time::now();
	x[0] = -1.442;
	x[1] = -4.558;
	x[2] = 2.203;
	x[3] = 5.288;
	y[0] = -3.647;
	y[1] = 0.517;
	y[2] = 5.181;
	y[3] = 2.461;

	int i = 0;

	while(i < 4) {
		goal.target_pose.pose.position.x = x[i];
		goal.target_pose.pose.position.y = y[i];
		goal.target_pose.pose.orientation.w = 1;
		ac.sendGoal(goal);

		ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		i++;
	}
}
  
  return 0;
}
