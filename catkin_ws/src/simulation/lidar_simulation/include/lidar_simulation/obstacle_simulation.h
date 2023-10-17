#ifndef OBSTACLE_SIMULATION_H
#define OBSTACLE_SIMULATION_H

#include "ros/ros.h"
#include "string"

#include "geometry_msgs/Polygon.h"
#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/InteractiveMarkerControl.h"
#include "interactive_markers/interactive_marker_server.h"
#include "interactive_markers/menu_handler.h"
#include "visualization_msgs/Marker.h"

using namespace std;

namespace vehicle_simulation {
class ObstacleSimulation{
public:
  ObstacleSimulation();
  ~ObstacleSimulation(){}

  void run();

private:
  void getInnerPoint(geometry_msgs::Polygon& footprint, vector<geometry_msgs::Point>& point_list, double resolution);
  bool pnpoly(geometry_msgs::Polygon& footprint, float& x, float& y);
  void getFootprintsRange(geometry_msgs::Polygon& footprint, float& min_x, float& max_x, float& min_y, float& max_y);
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void menuFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void initMenu();

  string obstacle_id_;
  string inter_marker_id_;
  double resolution_;

  bool transform_flag_;
  ros::NodeHandle nh_;

  ros::ServiceClient client_;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  vector<geometry_msgs::Polygon> obstacle_footprints_;

  interactive_markers::MenuHandler menu_handler_;

};
}


#endif // OBSTACLE_SIMULATION_H
