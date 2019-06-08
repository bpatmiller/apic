#include "grid.h"
#include <algorithm>
#include <iostream>

#define FLUID_CELL 0
#define AIR_CELL 1
#define SOLID_CELL 2

// ensure no particle moves more than grid.h
float Grid::CFL() {
  float maxsq = glm::pow(u.infnorm(), 2.0f) + glm::pow(w.infnorm(), 2.0f) +
                glm::pow(v.infnorm(), 2.0f);
  maxsq = std::max(maxsq, 1e-16f);
  return std::max(1e-3f,h / std::sqrt(maxsq));
}

void Grid::add_gravity(float dt) {
  float g = dt * gravity;
  for (int i = 0; i < v.size; i++) {
    v.data[i] += g;
  }
}

void Grid::compute_phi() {
  // init
  float big = phi.sx + phi.sy + phi.sz + 3;
  for (int i = 0; i < big; i++) {
    phi.data[i] = big;
  }
  for (int i = 1; i < phi.sx - 1; i++) {
    for (int j = 1; j < phi.sy - 1; j++) {
      for (int k = 1; k < phi.sz - 1; k++) {
        if (marker(i, j, k) == FLUID_CELL)
          phi(i, j, k) = -0.5;
      }
    }
  }

  for (int i = 0; i < 2; i++) {
    sweep_phi();
  }
}

void Grid::sweep_phi() {
  // for (int i = 1; i<phi.sx; i++) {
  //   for (int j = 1; j<phi.sy; j++) {
  //     for (int k = 1; k<phi.sz; k++) {
  //       if (marker(i,j,k)!= FLUID_CELL)
  //         solve_phi(phi(i-1,j,k))
  //     }
  //   }
  // }
}
void Grid::extend_velocity() {}
void Grid::enforce_boundary() {}
void Grid::project() {}