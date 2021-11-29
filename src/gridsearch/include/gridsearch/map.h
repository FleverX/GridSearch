#ifndef _MAP_H_
#define _MAP_H_
#include <math.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>

namespace planning {
namespace GridSearch {

class Map {
 public:
  Map() = delete;
  Map(const Map * map);
  Map(int _size_x, int _size_y);
  ~Map();

 public:
  void SetMap(const std::vector<int>& vec_in);
  bool SetCost(int x, int y, int cost);
  bool GetCost(int x, int y, int& cost);
  bool GetCost(int idx, int& cost);
  bool IsInMap(int index);
  bool GetIndexInMap(int x, int y, int& index);
  bool GetCellInWorld(int index, int& x, int& y);
    int GetCost(int idx);
  int GetSizeInX();
  int GetSizeInY();

 private:
  int size_x_, size_y_;
  int* cell_array_;
};

}  // namespace GridSearch
}  // namespace planning

#endif  //_MAP_H_
