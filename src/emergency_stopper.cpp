#include <tf/transform_datatypes.h>
#include "emergency_stopper/emergency_stopper.h"


EmergencyStopper::EmergencyStopper(ros::NodeHandle& nh) 
    : nh(nh),
    privNh("~") {

	privNh.param<std::string>("scan_topic", scanTopic, "/scan");
	privNh.param<std::string>("winner_topic", winnerTopic, "/winner");
	privNh.param<std::string>("multiplier_topic", multiplierTopic, "/multiplier");
	privNh.param<bool>("use_sonar", sonarUse, false);
	privNh.param<double>("base_distance", baseDistance, 0.35f);
	privNh.param<double>("stop_speed", stopSpeed, 0.2f);
	privNh.param<double>("stopper_width", stopperWidth, 60.0f);
	privNh.param<std::string>("joy_topic", joyTopic, "/joy");
	scanReceived = false;
	winner.data = false;
	joyReceived = false;
	multiplier.data = 1.0;
	loopHz = 50;
	if(sonarUse == false)
	{
		rangerSub = nh.subscribe(scanTopic, 1, &EmergencyStopper::laserCallback, this);
	}
	else
	{
		rangerSub = nh.subscribe(scanTopic, 1, &EmergencyStopper::sonarCallback, this); 
	}
	joySub = nh.subscribe(joyTopic, 1, &EmergencyStopper::joyCallback, this);
	winnerSub = nh.subscribe(winnerTopic, 1, &EmergencyStopper::winnerCallback, this);
	multiplierSub = nh.subscribe(multiplierTopic, 1, &EmergencyStopper::multiplierCallback, this);
	cmdVelListenerSub = nh.subscribe("/listener/cmd_vel", 1, &EmergencyStopper::cmdVelListenerCallback, this);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	emergency_stop_pub = nh.advertise<std_msgs::Bool>("/emergency_stop", 1);
	move_cmd.linear.x = 0.0;
	move_cmd.angular.z = 0.0;
	received_cmd.linear.x = 0.0;
	received_cmd.angular.z = 0.0;
	cmdReceived = false;
		    

	ROS_INFO("[emergency_stopper] Initialized.");
}

EmergencyStopper::~EmergencyStopper() {
 	ROS_INFO("[emergency_stopper] Destroyed.");
}

void EmergencyStopper::joyCallback(const sensor_msgs::Joy joyMessage) {
	joyResult = joyMessage;
	joyReceived = true;
}

void EmergencyStopper::laserCallback(const sensor_msgs::LaserScan::ConstPtr& inputScan) {
    scan = *inputScan;
	scanReceived = true;
}

void EmergencyStopper::sonarCallback(const p2os_msgs::SonarArray sonarData) {
    sonarScan = sonarData.ranges;
	scanReceived = true;
}

void EmergencyStopper::winnerCallback(const std_msgs::Bool inputWinner) {
    winner = inputWinner;
}

void EmergencyStopper::multiplierCallback(const std_msgs::Float32 inputMultiplier) {

    multiplier = inputMultiplier;
}
void EmergencyStopper::cmdVelListenerCallback(const geometry_msgs::Twist navData) {
    received_cmd = navData;
	cmdReceived = true;
}


void EmergencyStopper::spinOnce() {
	ros::Rate rate(loopHz);
	result.data = false;
	if (scanReceived == true && cmdReceived == true)
	{
		
		
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
		if (sonarUse == false)
		{
			for (int i = ((scan.ranges.size()/2) - stopperWidth); i < 0.0f + ((scan.ranges.size()/2) + 60); i++)
			{
				float allowedDistance = baseDistance + (0.15 - (0.15*(fabs(((0.0f + scan.ranges.size())/2.0f) - i)/((0.0f + scan.ranges.size())/2.0f))));
				if (scan.ranges[i] < allowedDistance)
				{
					result.data = true;
					emergency_stop_pub.publish(result);
					move_cmd.linear.x = stopSpeed;
					//cmd_vel_pub.publish(move_cmd);
				}
			
			}
		}
		else
		{
			for (int i = 1.0; i < 7.0f; i++)
			{
				float allowedDistance = baseDistance + (0.15f - (0.15f*(fabs(((0.0f + 8.0f)/2.0f) - i)/((0.0f + 8.0f)/2.0f))));
				if (sonarScan[i] < allowedDistance)
				{
					ROS_INFO("[emergency_stopper] STOPPED! allowed: %f scan %d: %f", allowedDistance, i, sonarScan[i]);
					result.data = true;
					emergency_stop_pub.publish(result);
					move_cmd.linear.x = stopSpeed;
					//cmd_vel_pub.publish(move_cmd);
				}
				else
				{
					//ROS_INFO("[emergency_stopper] allowed: %f scan %d: %f", allowedDistance, i, sonarScan[i]);
				}
			
			}
		}
		if (joyResult.buttons[2] == 1 && joyReceived == true)
		{
					result.data = true;
					emergency_stop_pub.publish(result);
					move_cmd.linear.x = stopSpeed;
		}
		else if (result.data == false)
		{
			/*if (move_cmd.linear.x == -0.31909)
			{
				move_cmd.linear.x = 0.0;
			}
			if (move_cmd.linear.x >= 0.0)
			{
				move_cmd.linear.x = (move_cmd.linear.x)*(multiplier.data);
				
			}*/
			move_cmd.linear.x = (received_cmd.linear.x)*(multiplier.data);
			move_cmd.angular.z = received_cmd.angular.z;
			
		}
		
		cmd_vel_pub.publish(move_cmd);
	}
	else
	{
		/*if (scanReceived == false)
		{
			ROS_INFO("[emergency_stopper] Awaiting scan.");
		}
		else
		{
			ROS_INFO("[emergency_stopper] Awaiting move command.");
		}*/
	}	
	emergency_stop_pub.publish(result);
	rate.sleep();
  	ros::spinOnce();
}

void EmergencyStopper::spin() {
  ros::Rate rate(loopHz);
  while (ros::ok()) {
    spinOnce();
  }
}
