#include "iostream"
#include "nav_msgs/GetMap.h"

#include "lidar_simulation/lidar_simulation.h"

namespace vehicle_simulation {

LidarSimulation::LidarSimulation(){
  ros::NodeHandle private_node("~");
  private_node.param("min_angle", min_angle_, -M_PI);
  private_node.param("max_angle", max_angle_, M_PI);
  private_node.param("min_distance", min_dis_, 0.15);
  private_node.param("max_distance", max_dis_, 6.00);
  private_node.param("size", point_size_, 400);
  private_node.param("rate", rate_, 10);

  private_node.param("global_frame", global_frame_, string("map"));
  private_node.param("lidar_frame", lidar_frame_, string("lidar"));
  step_ = (max_angle_ - min_angle_) / (point_size_ - 1);
}

LidarSimulation::LidarSimulation(double min_angle, double max_angle, double min_dis, double max_dis, int point_size, int rate):
  min_angle_(min_angle), max_angle_(max_angle),
  min_dis_(min_dis), max_dis_(max_dis),
  point_size_(point_size),rate_(rate)
{
  step_ = (max_angle_ - min_angle_) / (point_size_ - 1);
}

LidarSimulation::~LidarSimulation(){}

void LidarSimulation::updateMap(ros::ServiceClient &client){
  ros::Rate fetch_rate(1);
  nav_msgs::GetMap message;
  while (true){
    if (client.call(message)){
      map_.initMap(message.response.map);
      ROS_INFO("Get map! size : %d x %d", message.response.map.info.height, message.response.map.info.width);
      break;
    }else{
      ROS_INFO("Waiting for the Map!");
    }
    fetch_rate.sleep();
  }
}

void LidarSimulation::getPose(tf::StampedTransform& transform, double& start_angle, double& reverse){
  double roll, pitch, yaw;
  tf_.lookupTransform(global_frame_, lidar_frame_, ros::Time(), transform);
  transform.getBasis().getRPY(roll, pitch, yaw);
  if (pow(roll,2) + pow(pitch,2) > esp){
    start_angle = yaw + max_angle_;
    reverse = -1.0;
  }else{
    start_angle = yaw + min_angle_;
    reverse = 1.0;
  }
}

void LidarSimulation::initLaserScan(sensor_msgs::LaserScan &laser_scan){
  laser_scan.header.frame_id = lidar_frame_;
  laser_scan.angle_min = min_angle_;
  laser_scan.angle_max = max_angle_;
  laser_scan.angle_increment = step_;
  laser_scan.time_increment = 1 / point_size_ / rate_;
  laser_scan.range_min = min_dis_;
  laser_scan.range_max = max_dis_;
}

void LidarSimulation::getFrame(sensor_msgs::LaserScan &laser_scan){
  tf::StampedTransform transform;
  double start_angle, reverse, angle;
  float start_wx, start_wy, end_wx, end_wy, dis;
  laser_scan.header.stamp = ros::Time::now();
  getPose(transform, start_angle, reverse);
  start_wx = (float)transform.getOrigin().getX();
  start_wy = (float)transform.getOrigin().getY();
  laser_scan.ranges.clear();
  for (int i=0; i<point_size_; i++){
    angle = start_angle + i * reverse * step_;
    map_.distance(start_wx, start_wy, angle, end_wx, end_wy);
    dis = sqrt(pow(end_wx - start_wx, 2) + pow(end_wy - start_wy, 2));
    if (dis <= max_dis_ && dis >= min_dis_){
      laser_scan.ranges.push_back(dis);
    }else{
      laser_scan.ranges.push_back(numeric_limits<float>::infinity());
    }
  }
}

bool LidarSimulation::obstacleHandleServer(lidar_simulation::Obstacle::Request &req, lidar_simulation::Obstacle::Response &res){
  if (req.type == req.TRANSFORM){
    map_.transformObstacle(req.obstacle_id, req.transform[0], req.transform[1], req.transform[2]);
  }else if (req.type == req.NEW){
    map_.insertObstacle(req.obstacle_id, req.vertex, req.transform[0], req.transform[1], req.transform[2]);
  }else if (req.type == req.DELETE){
    map_.deleteObstacle(req.obstacle_id);
  }else{
    return false;
  }
  map_.updateObstacleData();
  return true;
}

void LidarSimulation::pubLaserCallback(const ros::TimerEvent& event){
  sensor_msgs::LaserScan laserscan;
  initLaserScan(laserscan);
  getFrame(laserscan);
  pub_.publish(laserscan);
}

void LidarSimulation::run(){
  pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);
  client_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
  updateMap(client_);
  tf_.waitForTransform(global_frame_, lidar_frame_, ros::Time(), ros::Duration(1.0));
  obstacle_service_ = nh_.advertiseService("/obstacle_handle", &LidarSimulation::obstacleHandleServer, this);
  pub_laser_timer_ = nh_.createTimer(ros::Duration(1.0/rate_), &LidarSimulation::pubLaserCallback, this);
  ros::spin();
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_simulation");
  vehicle_simulation::LidarSimulation lidar;
  lidar.run();
  return 0;
}
