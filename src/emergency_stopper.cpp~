#include <tf/transform_datatypes.h>
#include "emergency_stopper/emergency_stopper.h"


EmergencyStopper::EmergencyStopper(ros::NodeHandle& nh) 
    : nh(nh),
    privNh("~") {

	privNh.param<std::string>("scan_topic", scanTopic, "/scan");
	privNh.param<std::string>("winner_topic", winnerTopic, "/winner");
	scanReceived = false;
	winner.data = false;
	loopHz = 10;
	laserSub = nh.subscribe(scanTopic, 1, &EmergencyStopper::laserCallback, this);
	winnerSub = nh.subscribe(winnerTopic, 1, &EmergencyStopper::winnerCallback, this);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	emergency_stop_pub = nh.advertise<std_msgs::Bool>("/emergency_stop", 1);
	//scrubbed_laser_pub = nh.advertise<sensor_msgs::LaserScan>(scrubbedScanTopic, 1);
	move_cmd.linear.x = 0.0;
	move_cmd.angular.z = 0.0;
		    

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


void EmergencyStopper::spinOnce() {
	if (scanReceived == true && winner.data == true)
	{
		
		result.data = false;
		for (int i = 0; i < scan.ranges.size(); i++)
		{
			float allowedDistance = 0.4 + (0.2 - (0.2*(fabs(90 - i)/90)));
			if (scan.ranges[i] < allowedDistance)
			{
				result.data = true;
				emergency_stop_pub.publish(result);
				cmd_vel_pub.publish(move_cmd);
			}
		}
		if (result.data == false)
		{
			emergency_stop_pub.publish(result);
		}
	}
	else
	{
		ROS_INFO("[emergency_stopper] Awaiting scan.");
	}	


  	ros::spinOnce();
}

void EmergencyStopper::spin() {
  ros::Rate rate(loopHz);
  while (ros::ok()) {
    spinOnce();
  }
}
