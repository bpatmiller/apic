#include "grid.h"
#include <algorithm>
#include <iostream>

// ensure no particle moves more than
// inspiration from Stanford APIC assignment
float Grid::CFL() {
  float max = std::max(
      gravity * h,
      glm::pow(u.infnorm() + w.infnorm() + glm::pow(v.infnorm(), 2.0f), 2.0f));
  max = std::max(max, 1e-16f);
  // in case of 0 velocity field
  return std::min(h / std::sqrt(max), 0.1f);
}

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