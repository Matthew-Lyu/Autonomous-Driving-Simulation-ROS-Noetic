#ifndef LIDAR_SIMULATION_H
#define LIDAR_SIMULATION_H

#include "ros/ros.h"

#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"

#include "lidar_simulation/Obstacle.h"
#include "lidar_simulation/static_map.h"

#define esp 1e-6

using namespace std;

namespace vehicle_simulation {

class LidarSimulation {
public:
  LidarSimulation();
  LidarSimulation(double min_angle, double max_angle, double min_dis, double max_dis, int point_size, int rate);
  ~LidarSimulation();

  void updateMap(ros::ServiceClient& client);
  void getPose(tf::StampedTransform& transform, double& start_angle, double& reverse);
  void getFrame(sensor_msgs::LaserScan& laser_scan);

  void run();
private:
  void initLaserScan(sensor_msgs::LaserScan& laser_scan);
  void pubLaserCallback(const ros::TimerEvent& event);
  bool obstacleHandleServer(lidar_simulation::Obstacle::Request &req, lidar_simulation::Obstacle::Response &res);


  string global_frame_, lidar_frame_;
  double min_angle_, max_angle_;
  double min_dis_, max_dis_, step_;
  int point_size_;
  int rate_;

  tf::TransformListener tf_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::ServiceClient client_;
  vehicle_simulation::StaticMap map_;

  ros::Timer pub_laser_timer_;
  ros::ServiceServer obstacle_service_;

};


}


#endif // LIDAR_SIMULATION_H
