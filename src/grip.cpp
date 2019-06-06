#include "grid.h"

void Grid::add_gravity(float dt) {
  float g = dt * gravity;
  for (int i = 0; i < v.size; i++) {
    v.data[i] += g;
  }
}

void Grid::compute_phi() {}
void Grid::extend_velocity() {}
void Grid::enforce_boundary() {}
void Grid::project() {}