#include <tf/transform_datatypes.h>
#include "emergency_stopper/emergency_stopper.h"


EmergencyStopper::EmergencyStopper(ros::NodeHandle& nh) 
    : nh(nh),
    privNh("~") {

	privNh.param<std::string>("scan_topic", scanTopic, "/scan");
	privNh.param<std::string>("winner_topic", winnerTopic, "/winner");
	privNh.param<std::string>("multiplier_topic", multiplierTopic, "/multiplier");
	scanReceived = false;
	winner.data = false;
	multiplier.data = 1.0;
	loopHz = 50;
	laserSub = nh.subscribe(scanTopic, 1, &EmergencyStopper::laserCallback, this);
	winnerSub = nh.subscribe(winnerTopic, 1, &EmergencyStopper::winnerCallback, this);
	multiplierSub = nh.subscribe(multiplierTopic, 1, &EmergencyStopper::multiplierCallback, this);
	cmdVelListenerSub = nh.subscribe("/listener/cmd_vel", 1, &EmergencyStopper::cmdVelListenerCallback, this);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	emergency_stop_pub = nh.advertise<std_msgs::Bool>("/emergency_stop", 1);
	move_cmd.linear.x = 0.0;
	move_cmd.angular.z = 0.0;
	cmdReceived = false;
		    

	ROS_INFO("[emergency_stopper] Initialized.");
}

EmergencyStopper::~EmergencyStopper() {
 	ROS_INFO("[emergency_stopper] Destroyed.");
}


void EmergencyStopper::laserCallback(const sensor_msgs::LaserScan::ConstPtr& inputScan) {
    scan = *inputScan;
	scanReceived = true;
}

void EmergencyStopper::winnerCallback(const std_msgs::Bool inputWinner) {
    winner = inputWinner;
}

void EmergencyStopper::multiplierCallback(const std_msgs::Float32 inputMultiplier) {

    multiplier = inputMultiplier;
}
void EmergencyStopper::cmdVelListenerCallback(const geometry_msgs::Twist navData) {
    move_cmd = navData;
	cmdReceived = true;
}


void EmergencyStopper::spinOnce() {
	ros::Rate rate(loopHz);
	if (scanReceived == true && cmdReceived == true)
	{
		
		result.data = false;
		/*for (int i = 0; i < scan.ranges.size(); i++)
		{
			float allowedDistance = 0.3 + (0.2 - (0.2*(fabs(90 - i)/90)));
			if (scan.ranges[i] < allowedDistance)
			{
				result.data = true;
				emergency_stop_pub.publish(result);
				cmd_vel_pub.publish(move_cmd);
			}
		}*/

		//for (int i = ((0.0f + scan.ranges.size())/4.0f); i < (3.0f*((0.0f + scan.ranges.size())/4.0f)); i++)
		for (int i = 0.0; i < 0.0f + scan.ranges.size(); i++)
		{
			float allowedDistance = 0.35 + (0.15 - (0.15*(fabs(((0.0f + scan.ranges.size())/2.0f) - i)/((0.0f + scan.ranges.size())/2.0f))));
			if (scan.ranges[i] < allowedDistance)
			{
				result.data = true;
				emergency_stop_pub.publish(result);
				move_cmd.linear.x = -0.31909;
				cmd_vel_pub.publish(move_cmd);
			}
			
		}
		if (result.data == false)
		{
			if (move_cmd.linear.x == -0.31909)
			{
				move_cmd.linear.x = 0.0;
			}
			if (move_cmd.linear.x > 0.0)
			{
				move_cmd.linear.x = (move_cmd.linear.x)*(multiplier.data);
			}
			
			cmd_vel_pub.publish(move_cmd);
			emergency_stop_pub.publish(result);
		}
	}
	else
	{
		result.data = false;
		emergency_stop_pub.publish(result);
		/*if (scanReceived == false)
		{
			ROS_INFO("[emergency_stopper] Awaiting scan.");
		}
		else
		{
			ROS_INFO("[emergency_stopper] Not in winner state.");
		}*/
	}	

	rate.sleep();
  	ros::spinOnce();
}

void EmergencyStopper::spin() {
  ros::Rate rate(loopHz);
  while (ros::ok()) {
    spinOnce();
  }
}
