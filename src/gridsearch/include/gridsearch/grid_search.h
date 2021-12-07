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

enum class SEARCH_METHOD {
  Dijkstra,
  Greedy,
  Astar,
};

struct SimpleNode {
  SimpleNode(){};
  SimpleNode(int _x, int _y) : x(_x), y(_y){};
  ~SimpleNode(){};

  int x;
  int y;
};

struct Node {
  Node() {};
  Node(int _x, int _y) : x(_x), y(_y){};
  Node(int _x, int _y, int _gc, int _hc, STATUS _status = STATUS::IDLE)
      : x(_x), y(_y), gc(_gc), hc(_hc), status(_status){};

  int x, y;
  STATUS status = STATUS::IDLE;
  int gc = 0;
  int hc = 0;
  int fc = 0;

  bool operator>(const Node& right) const { return fc > right.fc; }

  Node* parent = NULL;
};

class GridSearch {
 public:
  GridSearch();
  ~GridSearch();

 public:
  bool MakePlan(const SimpleNode& start_node, const SimpleNode& goal_node,
                const Map& search_map);

  bool SetStartNode(int x, int y, Node& start_node);
  bool SetGoalNode(int x, int y, Node& goal_node);
  bool SearchPath(const Node& start, const Node& goal);
  bool SetSearchMethod(const std::string search_method) {
    if (search_method == "Dijkstra") {
      search_method_ = SEARCH_METHOD::Dijkstra;
    } else if (search_method == "Greedy") {
      search_method_ = SEARCH_METHOD::Greedy;
    } else if (search_method == "Astar") {
      search_method_ = SEARCH_METHOD::Astar;
    } else {
      return false;
    }
    return true;
  }
  std::vector<SimpleNode> GetPath() const { return result_path_; }

 private:
  bool IsReachGoal(const Node& current_node, const Node& goal_node);
  void SetPath(Node& goal_node);
  void InitializeMap();

 private:
  Map search_map_;
  std::vector<SimpleNode> result_path_;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list_;
  std::vector<std::vector<Node>> nodes_;  // nodes_[i][j], i in y or height
                                          // driection, j in x or width
                                          // direction
  SEARCH_METHOD search_method_{SEARCH_METHOD::Dijkstra};
};

}  // namespace GridSearch
}  // namespace planning

#endif