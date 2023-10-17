#ifndef STATIC_MAP_H
#define STATIC_MAP_H

#include "map"
#include "vector"

#include "tf/tf.h"
#include "geometry_msgs/Polygon.h"
#include "nav_msgs/OccupancyGrid.h"

#define ERROR          -2
#define UNKNOWN_SPACE  -1
#define FREE_SPACE     0
#define OBS_SPACE      100

using namespace std;

struct Point
{
    float x;
    float y;
};

typedef vector<Point> footprint;
typedef vector<footprint> footprint_list;
typedef map<string, footprint_list> footprints_map;
typedef map<string, footprint> obstacle_points_map;

namespace vehicle_simulation {

class StaticMap
{
public:
  StaticMap(){}
  ~StaticMap();

  void initMap(nav_msgs::OccupancyGrid& grid_map);

  void mapToWorld(int mx, int my, float& wx, float& wy);
  void wordToMap(float wx, float wy, int& mx, int& my);

  char getValue(int mx, int my);
  char getValue(float wx, float wy);

  void setObstacleValue(int mx, int my, char value);
  void setObstacleValue(float wx, float wy, char value);

  void distance(int start_x, int start_y, double theta, int& end_x, int& end_y);
  void distance(float start_x, float start_y, double theta, float& end_x, float& end_y);

  double normalAngle(double theta);

  void insertObstacle(string obstacle_name, vector<geometry_msgs::Polygon> polygon_list, float x, float y, float yaw);
  void transformObstacle(string obstacle_name, float x, float y, float yaw);
  void deleteObstacle(string obstacle_name);

  void updateObstacleData();

private:
  void getObstacleRange(footprint& footprint, float& min_x, float& max_x, float& min_y, float& max_y);
  bool pnpoly(footprint& footprint, float& x, float& y);
  void getObstaclePoints(footprint& f, footprint& obs_points);
  void transformFootprint(footprint_list& source, footprint& target, float x, float y, float yaw);

  bool limit_range(int& mx, int& my);
  int size_x_, size_y_;
  double resolution_;
  double origin_x_, origin_y_;
  unsigned int data_length_;
  char* map_data_;
  char* obstacle_data_;

  bool exist_obstacle;
  footprints_map footprints_map_;
  obstacle_points_map obstacle_points_map_;

};

inline bool StaticMap::limit_range(int &mx, int &my){
  bool x_flag = (mx >= size_x_) || (mx <= 0);
  mx = x_flag ? std::max(std::min(mx, size_x_), 0) : mx;
  bool y_flag = (my >= size_y_) || (my <= 0);
  my = y_flag ? std::max(std::min(my, size_y_), 0) : my;
  return x_flag || y_flag;
}

}
#endif // STATIC_MAP_H
