#include <memory>
#include <vector>
#include "gridsearch/grid_search.h"
#include "gridsearch/map.h"
using namespace planning::GridSearch;

int main(int argc, char** argv) {
  // set map in vector [9 * 10]
  std::vector<int> test_map{0, -1, 0, 0, 0, 2, 1, 1, 0, -1,
                            0, 1,  3, 0, 0, 0, 1, 2, 3, 0};
  int size_x = 4;
  int size_y = 5;
  SimpleNode start_node(0, 0);
  SimpleNode goal_node(3, 4);
  // calc map
  Map grid_map(size_x, size_y);  // size_x = 9, size_y = 10
  if (test_map.size() != size_x * size_y) return 0;
  grid_map.SetMap(test_map);
  // initialize grid search planner
  std::shared_ptr<GridSearch> grid_search_planner =
      std::make_shared<GridSearch>();
  std::vector<SimpleNode> result_path;
  // now offer Dijkstra, Greedy, Astar
  // std::string method{"Dijkstra"};
  // std::string method{"Greedy"};
  std::string method{"Astar"};

  if (!grid_search_planner->SetSearchMethod(method)) return 0;
  if (grid_search_planner->MakePlan(start_node, goal_node, grid_map)) {
    result_path = grid_search_planner->GetPath();
  } else {
    std::cout << "search path fail!" << std::endl;
  }
  for (size_t i = 0; i < result_path.size(); ++i) {
    std::cout << "i: " << i << " x： " << result_path.at(i).x
              << " y: " << result_path.at(i).y << std::endl;
  }
  return 0;
}
