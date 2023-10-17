#include "lidar_simulation/static_map.h"

namespace vehicle_simulation {

StaticMap::~StaticMap(){
  if (data_length_ > 0){
    delete [] map_data_;
  }
}

void StaticMap::initMap(nav_msgs::OccupancyGrid& grid_map){
  resolution_ = grid_map.info.resolution;
  origin_x_ = grid_map.info.origin.position.x;
  origin_y_ = grid_map.info.origin.position.y;
  size_x_ = grid_map.info.width;
  size_y_ = grid_map.info.height;
  data_length_ = size_x_ * size_y_;
  map_data_ = new char[data_length_];
  obstacle_data_ = new char[data_length_];
  for (int i=0; i<data_length_; i++){
    map_data_[i] = grid_map.data.at(i);
  }
  exist_obstacle = false;
}

void StaticMap::mapToWorld(int mx, int my, float &wx, float &wy){
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

void StaticMap::wordToMap(float wx, float wy, int &mx, int &my){
  mx = (int)((wx - origin_x_) / resolution_);
  my = (int)((wy - origin_y_) / resolution_);
}

char StaticMap::getValue(int mx, int my){
  int index = my * size_x_ + mx;
  if (index < 0 && index > data_length_){
    return ERROR;
  }else{
    if (exist_obstacle){
      return obstacle_data_[index] == OBS_SPACE ? OBS_SPACE : map_data_[index];
    }else{
      return map_data_[index];
    }

  }
}

char StaticMap::getValue(float wx, float wy){
  int mx, my;
  wordToMap(wx, wy, mx, my);
  return getValue(mx, my);
}

void StaticMap::distance(int start_x, int start_y, double theta, int& end_x, int& end_y){
  theta = normalAngle(theta);
  double k, e=-0.5;
  int major_step, minor_step;
  if (limit_range(start_x, start_y)){
    end_x = start_x;
    end_y = start_y;
    return;
  }else if (std::abs(theta) <= (0.25 * M_PI) || std::abs(theta) >= (0.75 * M_PI)){
    k = std::abs(std::tan(theta));
    major_step = std::abs(theta) <= (0.25 * M_PI) ? 1 : -1;
    minor_step = theta >= 0 ? 1 : -1;
    while (true){
      start_x += major_step;
      e += k;
      if (e >= 0){
        start_y += minor_step;
        e -= 1;
      }
      if (limit_range(start_x, start_y) || getValue(start_x, start_y) == OBS_SPACE || getValue(start_x, start_y) == UNKNOWN_SPACE){
        end_x = start_x;
        end_y = start_y;
        return;
      }
    }
  }else{
    k = std::abs(tan(0.5 * M_PI - theta));
    major_step = theta > 0 ? 1 : -1;
    minor_step = std::abs(theta) <= (0.5 * M_PI) ? 1 : -1;
    while (true){
      start_y += major_step;
      e += k;
      if (e >= 0){
        start_x += minor_step;
        e -= 1;
      }
      if (limit_range(start_x, start_y) || getValue(start_x, start_y) == OBS_SPACE || getValue(start_x, start_y) == UNKNOWN_SPACE){
        end_x = start_x;
        end_y = start_y;
        return;
      }
    }
  }
}

void StaticMap::distance(float start_x, float start_y, double theta, float &end_x, float &end_y){
  int start_mx, start_my, end_mx, end_my;
  wordToMap(start_x, start_y, start_mx, start_my);
  distance(start_mx, start_my, theta, end_mx, end_my);
  mapToWorld(end_mx, end_my, end_x, end_y);
}


void StaticMap::getObstacleRange(footprint& footprint, float& min_x, float& max_x, float& min_y, float& max_y){
  if (footprint.size() > 0){
    min_x = max_x = footprint.at(0).x;
    min_y = max_y = footprint.at(0).y;
    for (int i=1; i<footprint.size(); i++){
      min_x = footprint.at(i).x < min_x ? footprint.at(i).x : min_x;
      max_x = footprint.at(i).x > max_x ? footprint.at(i).x : max_x;
      min_y = footprint.at(i).y < min_y ? footprint.at(i).y : min_y;
      max_y = footprint.at(i).y > max_y ? footprint.at(i).y : max_y;
    }
    min_x = std::max(min_x, (float)origin_x_);
    max_x = std::min(max_x, (float)(origin_x_ + size_x_ * resolution_));
    min_y = std::max(min_y, (float)origin_y_);
    max_y = std::min(max_y, (float)(origin_y_ + size_y_ * resolution_));
  }
}

bool StaticMap::pnpoly(footprint& footprint, float& x, float& y){
  int i,j;
  bool c = false;
  for (i=0, j=footprint.size()-1; i<footprint.size(); j = i++){
    if ( ( (footprint.at(i).y > y) != (footprint.at(j).y > y) ) &&
         (x < (footprint.at(j).x-footprint.at(i).x) * (y-footprint.at(i).y) / (footprint.at(j).y - footprint.at(i).y) + footprint.at(i).x) ){
      c = !c;
    }
  }
  return c;
}

void StaticMap::getObstaclePoints(footprint& f, footprint& obs_points){
  float min_x, max_x, min_y, max_y;
  getObstacleRange(f, min_x, max_x, min_y, max_y);
  for (float x = (std::ceil(min_x / resolution_) * resolution_); x <= max_x; x = x + resolution_/2){
    for (float y = (std::ceil(min_y / resolution_) * resolution_); y <= max_y; y = y + resolution_/2){
      if (pnpoly(f, x, y)){
        Point p; p.x = x; p.y = y;
        obs_points.push_back(p);
      }
    }
  }
}

void StaticMap::deleteObstacle(string obstacle_name){
  if (footprints_map_.find(obstacle_name) != footprints_map_.end()){
    footprints_map_.erase(obstacle_name);
  }

  if (obstacle_points_map_.find(obstacle_name) != obstacle_points_map_.end()){
    obstacle_points_map_.erase(obstacle_name);
  }

  if(obstacle_points_map_.size() == 0){
    exist_obstacle = false;
  }

}

void StaticMap::transformFootprint(footprint_list &source, footprint &target, float x, float y, float yaw){
  float cos_th = cos(yaw);
  float sin_th = sin(yaw);
  footprint f;
  for (int i=0; i<source.size(); i++){
    f.clear();
    for (int j=0; j<source[i].size(); j++){
      Point p;
      p.x =  x + (source[i][j].x * cos_th - source[i][j].y * sin_th);
      p.y =  y + (source[i][j].x * sin_th + source[i][j].y * cos_th);
      f.push_back(p);
    }
    getObstaclePoints(f, target);
  }
}

void StaticMap::transformObstacle(string obstacle_name, float x, float y, float yaw){
  footprints_map::iterator iter = footprints_map_.find(obstacle_name);
  if (iter == footprints_map_.end()){
    throw runtime_error("No specified obstacle");
  }

  if (obstacle_points_map_.find(obstacle_name) != obstacle_points_map_.end()){
    obstacle_points_map_.erase(obstacle_name);
  }
  footprint obs_points;
  transformFootprint(iter->second, obs_points, x, y, yaw);
  obstacle_points_map_[obstacle_name] = obs_points;
}

void StaticMap::insertObstacle(string obstacle_name, vector<geometry_msgs::Polygon> polygon_list, float x, float y, float yaw){
  deleteObstacle(obstacle_name);
  footprint_list obstacle_footprints;
  for (int i=0; i<polygon_list.size(); i++){
    footprint f;
    for (int j=0; j<polygon_list.at(i).points.size(); j++){
      Point p;
      p.x = polygon_list.at(i).points.at(j).x;
      p.y = polygon_list.at(i).points.at(j).y;
      f.push_back(p);
    }
    obstacle_footprints.push_back(f);
  }
  footprints_map_[obstacle_name] = obstacle_footprints;

  footprint obstacle_points;
  transformFootprint(obstacle_footprints, obstacle_points, x, y, yaw);
  obstacle_points_map_[obstacle_name] = obstacle_points;
  exist_obstacle = true;
}

void StaticMap::updateObstacleData(){
  int mx, my, index;
  memset(obstacle_data_, 0, data_length_);
  for (obstacle_points_map::iterator iter = obstacle_points_map_.begin(); iter!=obstacle_points_map_.end(); iter++){
    for (int i=0; i<(iter->second).size(); i++){
      wordToMap((iter->second)[i].x, (iter->second)[i].y, mx, my);
      index = my * size_x_ + mx;
      if (index >=0 && index <= data_length_){
        obstacle_data_[index] = OBS_SPACE;
      }
    }
  }
}

double StaticMap::normalAngle(double theta){
  if (theta > M_PI){
    theta = normalAngle(theta - 2*M_PI);
  }else if (theta <= -M_PI){
    theta = normalAngle(theta + 2*M_PI);
  }
  return theta;
}


}
