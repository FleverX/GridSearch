#include "gridsearch/grid_search.h"

namespace planning {
namespace GridSearch {

GridSearch::GridSearch(){};
GridSearch::~GridSearch(){};

bool GridSearch::MakePlan(const SimpleNode& start_node, const SimpleNode& goal_node, const Map& search_map) {
  search_map_ = search_map;

  InitializeMap();
  // {
  //   for (size_t i = 0; i < nodes_.size(); ++i) {
  //     for (size_t j = 0; j < nodes_[0].size(); ++j) {
  //       std::cout << "row: " << nodes_[i][j].y << std::endl;
  //       std::cout << "column: " << nodes_[i][j].x << std::endl;
  //     }
  //     std::cout << "-----------" << std::endl;
  //   }
  // }
  Node start_node_search, goal_node_search;
  if (!SetStartNode(start_node.x, start_node.y, start_node_search)) return false;
  if (!SetGoalNode(goal_node.x, goal_node.y, goal_node_search)) return false;
  if (SearchPath(start_node_search, goal_node_search)) return true;
  return false;
}

bool GridSearch::SetStartNode(int x, int y, Node& start_node) {
  // if value is -1, consider as obs, value larger, movement_cost is larger
  int cell_cost;
  search_map_.GetCost(x, y, cell_cost);
  if (!search_map_.GetCost(x, y, cell_cost) || cell_cost < 0) return false;
  start_node.x = x;
  start_node.y = y;
  nodes_[y][x].status = STATUS::OPEN;
  return true;
}

bool GridSearch::SetGoalNode(int x, int y, Node& goal_node) {
  int cell_cost;
  if (!search_map_.GetCost(x, y, cell_cost) || cell_cost < 0) return false;
  goal_node.x = x;
  goal_node.y = y;
  return true;
}

bool GridSearch::SearchPath(const Node& start, const Node& goal) {
  open_list_.push(start);
  while (!open_list_.empty()) {
    Node top_node;
    top_node = open_list_.top();
    open_list_.pop();
    nodes_[top_node.y][top_node.x].status = STATUS::CLOSED;

    if (IsReachGoal(top_node, goal)) {
      SetPath(top_node);
      std::cout << search_map_.GetValue() << std::endl;
      return true;
    }

    Node* current_node = &nodes_[top_node.y][top_node.x];

    // expand node in four directions
    int cur_idx;
    search_map_.GetIndexInMap(top_node.x, top_node.y, cur_idx);

    // rigth
    Node* next_node;
    int next_idx = cur_idx + 1;
    if (search_map_.IsInMap(next_idx)) {
      int mx, my;
      search_map_.GetCellInWorld(next_idx, mx, my);
      next_node = &nodes_[my][mx];
      if (next_node->status == STATUS::IDLE) {
        next_node->status = STATUS::OPEN;
        next_node->parent = current_node;
        switch (search_method_) {
          case SEARCH_METHOD::Dijkstra: {
            next_node->gc += search_map_.GetCost(next_idx);
          } break;
          case SEARCH_METHOD::Greedy: {
            next_node->hc = abs(next_node->x - goal.x) + abs(next_node->y - goal.y);
          } break;
          case SEARCH_METHOD::Astar: {
            next_node->gc += search_map_.GetCost(next_idx);
            next_node->hc = abs(next_node->x - goal.x) + abs(next_node->y - goal.y);
          } break;
          default:
            break;
        }
        Node temp_node;
        temp_node = *next_node;
        open_list_.push(temp_node);
      } else if (next_node->status == STATUS::OPEN) {
        switch (search_method_) {
          case SEARCH_METHOD::Dijkstra: {
            double cur_gc = current_node->gc;
            double movement_cost = search_map_.GetCost(next_idx);  // move to right cell cost
            double total_cost = cur_gc + movement_cost;
            if (total_cost < next_node->gc) {
              next_node->gc = total_cost;
              next_node->parent = current_node;
            }
          } break;
          case SEARCH_METHOD::Greedy: {
          } break;
          case SEARCH_METHOD::Astar: {
            double cur_gc = current_node->gc;
            double movement_cost = search_map_.GetCost(next_idx);  // move to right cell cost
            double total_cost = cur_gc + movement_cost;
            if (total_cost < next_node->gc) {
              next_node->gc = total_cost;
              next_node->parent = current_node;
            };
          } break;
          default:
            break;
        }
      }
    }

    // left
    next_idx = cur_idx - 1;
    if (search_map_.IsInMap(next_idx)) {
      int mx, my;
      search_map_.GetCellInWorld(next_idx, mx, my);
      next_node = &nodes_[my][mx];
      if (next_node->status == STATUS::IDLE) {
        next_node->status = STATUS::OPEN;
        next_node->parent = current_node;
        switch (search_method_) {
          case SEARCH_METHOD::Dijkstra: {
            next_node->gc += search_map_.GetCost(next_idx);
          } break;
          case SEARCH_METHOD::Greedy: {
            next_node->hc = abs(next_node->x - goal.x) + abs(next_node->y - goal.y);
          } break;
          case SEARCH_METHOD::Astar: {
            next_node->gc += search_map_.GetCost(next_idx);
            next_node->hc = abs(next_node->x - goal.x) + abs(next_node->y - goal.y);
          } break;
          default:
            break;
        }
        Node temp_node;
        temp_node = *next_node;
        open_list_.push(temp_node);
      } else if (next_node->status == STATUS::OPEN) {
        switch (search_method_) {
          case SEARCH_METHOD::Dijkstra: {
            double cur_gc = current_node->gc;
            double movement_cost = search_map_.GetCost(next_idx);  // move to left cell cost
            double total_cost = cur_gc + movement_cost;
            if (total_cost < next_node->gc) {
              next_node->gc = total_cost;
              next_node->parent = current_node;
            }
          } break;
          case SEARCH_METHOD::Greedy: {
          } break;
          case SEARCH_METHOD::Astar: {
            double cur_gc = current_node->gc;
            double movement_cost = search_map_.GetCost(next_idx);  // move to left cell cost
            double total_cost = cur_gc + movement_cost;
            if (total_cost < next_node->gc) {
              next_node->gc = total_cost;
              next_node->parent = current_node;
            };
          } break;
          default:
            break;
        }
      }
    }

    // std::cout << "left: " << next_idx << std::endl;

    // top
    next_idx = cur_idx + search_map_.GetSizeInX();
    if (search_map_.IsInMap(next_idx)) {
      int mx, my;
      search_map_.GetCellInWorld(next_idx, mx, my);
      next_node = &nodes_[my][mx];
      // std::cout << "top+++++" << std::endl;
      if (next_node->status == STATUS::IDLE) {
        next_node->status = STATUS::OPEN;
        next_node->parent = current_node;
        switch (search_method_) {
          case SEARCH_METHOD::Dijkstra: {
            next_node->gc += search_map_.GetCost(next_idx);
          } break;
          case SEARCH_METHOD::Greedy: {
            next_node->hc = abs(next_node->x - goal.x) + abs(next_node->y - goal.y);
          } break;
          case SEARCH_METHOD::Astar: {
            next_node->gc += search_map_.GetCost(next_idx);
            next_node->hc = abs(next_node->x - goal.x) + abs(next_node->y - goal.y);
          } break;
          default:
            break;
        }
        Node temp_node;
        temp_node = *next_node;
        open_list_.push(temp_node);
      } else if (next_node->status == STATUS::OPEN) {
        switch (search_method_) {
          case SEARCH_METHOD::Dijkstra: {
            double cur_gc = current_node->gc;
            double movement_cost = search_map_.GetCost(next_idx);  // move to top cell cost
            double total_cost = cur_gc + movement_cost;
            if (total_cost < next_node->gc) {
              next_node->gc = total_cost;
              next_node->parent = current_node;
            }
          } break;
          case SEARCH_METHOD::Greedy: {
          } break;
          case SEARCH_METHOD::Astar: {
            double cur_gc = current_node->gc;
            double movement_cost = search_map_.GetCost(next_idx);  // move to top cell cost
            double total_cost = cur_gc + movement_cost;
            if (total_cost < next_node->gc) {
              next_node->gc = total_cost;
              next_node->parent = current_node;
            };
          } break;
          default:
            break;
        }
      }
    }

    // down
    next_idx = cur_idx - search_map_.GetSizeInX();
    if (search_map_.IsInMap(next_idx)) {
      int mx, my;
      search_map_.GetCellInWorld(next_idx, mx, my);
      next_node = &nodes_[my][mx];
      if (next_node->status == STATUS::IDLE) {
        next_node->status = STATUS::OPEN;
        next_node->parent = current_node;
        switch (search_method_) {
          case SEARCH_METHOD::Dijkstra: {
            next_node->gc += search_map_.GetCost(next_idx);
          } break;
          case SEARCH_METHOD::Greedy: {
            next_node->hc = abs(next_node->x - goal.x) + abs(next_node->y - goal.y);
          } break;
          case SEARCH_METHOD::Astar: {
            next_node->gc += search_map_.GetCost(next_idx);
            next_node->hc = abs(next_node->x - goal.x) + abs(next_node->y - goal.y);
          } break;
          default:
            break;
        }
        Node temp_node;
        temp_node = *next_node;
        open_list_.push(temp_node);
      } else if (next_node->status == STATUS::OPEN) {
        switch (search_method_) {
          case SEARCH_METHOD::Dijkstra: {
            double cur_gc = current_node->gc;
            double movement_cost = search_map_.GetCost(next_idx);  // move to down cell cost
            double total_cost = cur_gc + movement_cost;
            if (total_cost < next_node->gc) {
              next_node->gc = total_cost;
              next_node->parent = current_node;
            }
          } break;
          case SEARCH_METHOD::Greedy: {
          } break;
          case SEARCH_METHOD::Astar: {
            double cur_gc = current_node->gc;
            double movement_cost = search_map_.GetCost(next_idx);  // move to down cell cost
            double total_cost = cur_gc + movement_cost;
            if (total_cost < next_node->gc) {
              next_node->gc = total_cost;
              next_node->parent = current_node;
            };
          } break;
          default:
            break;
        }
      }
    }
  }  // while
  return false;
}

void GridSearch::InitializeMap() {
  if (&search_map_) {
    int size_in_x = search_map_.GetSizeInX();
    int size_in_y = search_map_.GetSizeInY();
    nodes_.resize(size_in_y);
    for (size_t i = 0; i < size_in_y; ++i) {
      nodes_.resize(size_in_x);
    }
    for (size_t i = 0; i < size_in_y; ++i) {
      for (size_t j = 0; j < size_in_x; ++j) {
        int idx = i * size_in_x + j;
        int cost;
        search_map_.GetCost(idx, cost);
        Node node(j, i);
        nodes_.at(i).emplace_back(node);
        if (cost < 0) nodes_[i][j].status = STATUS::OBS;
        if (search_method_ == SEARCH_METHOD::Dijkstra) nodes_[i][j].hc = 0;
        if (search_method_ == SEARCH_METHOD::Greedy) nodes_[i][j].gc = 0;
      }
    }
  }
}

bool GridSearch::IsReachGoal(const Node& current_node, const Node& goal_node) {
  // std::cout << "current: " << current_node.x << current_node.y
  //           << "goal: " << goal_node.x << goal_node.y << std::endl;

  if ((abs(current_node.x - goal_node.x) < 1) && (abs(current_node.y - goal_node.y) < 1)) return true;
  return false;
}

void GridSearch::SetPath(Node& goal_node) {
  Node* temp_node = &goal_node;
  while (temp_node != nullptr) {
    SimpleNode path_node;
    path_node.x = temp_node->x;
    path_node.y = temp_node->y;
    result_path_.emplace_back(path_node);
    temp_node = temp_node->parent;
  }
  std::reverse(result_path_.begin(), result_path_.end());
}

}  // namespace GridSearch
}  // namespace planning