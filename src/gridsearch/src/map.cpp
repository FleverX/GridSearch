#include "include/gridsearch/map.h"

namespace planning {
namespace GridSearch {

Map::Map(int _size_x, int _size_y) : size_x_(_size_x), size_y_(_size_y) {
  cell_array_ = new int[size_x_ * size_y_];
  memset(cell_array_, 0, size_x_ * size_y_ * sizeof(int));
}

Map::~Map() { delete[] cell_array_; }

Map::Map(const Map &map) {
    this->size_x_ = map.size_x_;
    this->size_y_ = map.size_y_;
    *this->cell_array_ = *map.cell_array_;
}

void Map::SetMap(const std::vector<int>& vec_in) {
  if (vec_in.empty()) {
    std::cout << "input map is empty" << std::endl;
    return;
  }
  for (size_t i = 0; i < vec_in.size(); ++i) {
    cell_array_[i] = vec_in.at(i);
  }
  return;
}

bool Map::SetCost(int x, int y, int cost) {
    int idx;
    if(!GetIndexInMap(x, y , idx)) return false;
    cell_array_ [idx] = cost;
    return true;
}
bool Map::GetCost(int x, int y, int& cost) {
    int idx;
    if(!GetIndexInMap(x, y, idx)) return false;
    cost = cell_array_[idx];
    return true;
}
bool Map::GetCost(int idx, int& cost) {
    if(idx < 0 || idx > size_x_ * size_y_) return false;
    cost = cell_array_[idx];
    return true;
}

bool Map::GetIndexInMap(int x, int y, &int index) {
    if(x >= 0 && x < size_x_ && y >= 0 && y < size_y_) {
        index = y * size_x_ + x;
        return true;
    }
    return false;
}

bool Map::GetCellInWorld(int index, int &x, int &y) {
    if(index >= 0 && index < size_x_ * size_y_) {
        y = index / size_x_;
        x = index % size_x_;
        return true;
    }
    return false;
}

int Map::GetSizeInX() {
    return size_x_; 
}

int Map::GetSizeInY() {
    return size_y_;
}

}  // namespace GridSearch
}  // namespace planning