#ifndef MAP_LASER_FILTER_H
#define MAP_LASER_FILTER_H

#include <ros/ros.h>
#include <cmath>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <p2os_msgs/GripperState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <p2os_msgs/SonarArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/Joy.h>
#include <autonomy_leds_msgs/Keyframe.h>
#include <vector>
#include <unistd.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Header.h>

#define PI 3.14159
#define TWO_PI 6.283185*/

class EmergencyStopper {
private:

	double loopHz;
	bool scanReceived;
	bool cmdReceived;
	bool sonarUse;
	std_msgs::Bool winner;
	std_msgs::Bool result;
	std_msgs::Float32 multiplier;

	std::string scanTopic;
	std::string winnerTopic;
	std::string multiplierTopic;
	std::string joyTopic;

	sensor_msgs::LaserScan scan;
	std::vector<double> sonarScan;
	geometry_msgs::Twist received_cmd;
	geometry_msgs::Twist move_cmd;

	sensor_msgs::Joy joyResult;

	bool joyReceived;
	
	double baseDistance;
	double stopSpeed;
	double stopperWidth;

  	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& inputScan);
	void sonarCallback(const p2os_msgs::SonarArray sonarData);
	void winnerCallback(const std_msgs::Bool inputWinner);
	void multiplierCallback(const std_msgs::Float32 inputMultiplier);
	void cmdVelListenerCallback(const geometry_msgs::Twist navData);
	void joyCallback(const sensor_msgs::Joy joyMessage);
	
protected:
	ros::NodeHandle nh;
	ros::NodeHandle privNh;
  

	ros::Publisher cmd_vel_pub;
	ros::Publisher emergency_stop_pub;

 	ros::Subscriber rangerSub;
	ros::Subscriber winnerSub;
	ros::Subscriber multiplierSub;
	ros::Subscriber cmdVelListenerSub;
	ros::Subscriber joySub;


public:
  EmergencyStopper(ros::NodeHandle& nh);
  ~EmergencyStopper();

  virtual void spin();
  virtual void spinOnce();

}; // class EmergencyStopper

#endif // MAP_LASER_FILTER_H
