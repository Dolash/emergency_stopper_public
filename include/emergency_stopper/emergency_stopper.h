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
#include <p2os_msgs/SonarArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/ColorRGBA.h>
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
	std_msgs::Bool winner;
	std_msgs::Bool result;

	std::string scanTopic;
	std::string winnerTopic;

	sensor_msgs::LaserScan scan;
	geometry_msgs::Twist move_cmd;
	

  	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& inputScan);
	void winnerCallback(const std_msgs::Bool inputWinner);

protected:
	ros::NodeHandle nh;
	ros::NodeHandle privNh;
  

	ros::Publisher cmd_vel_pub;
	ros::Publisher emergency_stop_pub;

 	ros::Subscriber laserSub;
	ros::Subscriber winnerSub;


public:
  EmergencyStopper(ros::NodeHandle& nh);
  ~EmergencyStopper();

  virtual void spin();
  virtual void spinOnce();

}; // class EmergencyStopper

#endif // MAP_LASER_FILTER_H
