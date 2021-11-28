#ifndef _GRID_SEARCH_H_
#define _GRID_SEARCH_H_

#include <ros/ros.h>
#include <algorithm>
#include <iostream>
#include <memory>
#include <queue>
#include <vector>
#include "map.h"

namespace planning {
namespace GridSearch {

enum class STATUS {
  IDLE,
  OPEN,
  CLOSED,
  OBS,
};

struct SimpleNode {
  SimpleNode(){};
  SimpleNode(int _x, int _y) : x(_x), y(_y){};
  ~SimpleNode(){};

  int x;
  int y;
};

struct Node {
  Node() = default;
  Node(int _x, int _y) : x(_x), y(_y){};
  Node(int _x, int _y, int _gc, int _hc, STATUS _status = STATUS::IDLE)
      : x(_x), y(_y), gc(_gc), hc(_hc), status(_status){};

  int x, y;
  STATUS status = STATUS::IDLE;
  int gc = 0;
  int hc = 0;
  int fc = gc + hc;

  bool operator>(const Node& right) const { return fc > right.fc; }

  Node* parent = NULL;
};

class GridSearch {
  GridSearch() = default;
  ~GridSearch();

 public:
  void MakePlan(const SimpleNode& start_node, const SimpleNode& goal_node,
                const std::shared_ptr<Map>& search_map);

  bool SetMapDimension(int size_x, int size_y);
  bool SetStartNode(int x, int y, Node& start_node);
  bool SetGoalNode(int x, int y, Node& goal_node);
  void SearchPath(const Node& start, const Node& goal);
  void SetSearchMethod(const std::string search_method) {
    search_method_ = search_method;
  }
  std::vector<SimpleNode> GetPath() const { return result_path_; }

 private:
  bool IsReachGoal(const& current_node, const& goal_node);
  void SetPath(const& goal_node);
  void InitializeMap();

 private:
  std::shared_ptr<Map> search_map_;
  std::vector<SimpleNode> result_path_;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list_;
  std::vector<std::vector<Node>> nodes_;  // nodes_[i][j], i in y or height
                                          // driection, j in x or width
                                          // direction
  std::string search_method_{""};
};

}  // namespace GridSearch
}  // namespace planning

#endif