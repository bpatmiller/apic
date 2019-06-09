#include "grid.h"
#include "Particle.h"
#include <algorithm>
#include <iostream>

// ensure no particle moves more than grid.h
float Grid::CFL() {
  float maxsq = glm::pow(u.infnorm(), 2.0f) + glm::pow(w.infnorm(), 2.0f) +
                glm::pow(v.infnorm(), 2.0f);
  maxsq = std::max(maxsq, 1e-16f);
  return std::max(1e-3f, h / std::sqrt(maxsq));
}

void Grid::add_gravity(float dt) {
  float g = dt * gravity;
  for (int i = 0; i < v.size; i++) {
    v.data[i] += g;
  }
}

void Grid::extend_velocity() {}
void Grid::enforce_boundary() {}
void Grid::project() {}

void Grid::compute_phi() {
  // init
  float big = phi.sx + phi.sy + phi.sz + 3;
  for (int i = 0; i < phi.size; i++) {
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
  // sweep in 8 directions

  // f, f, f
  for (int i = 1; i < phi.sx; i++) {
    for (int j = 1; j < phi.sy; j++) {
      for (int k = 1; k < phi.sz; k++) {
        if (marker(i, j, k) != FLUID_CELL)
          solve_phi(phi(i - 1, j, k), phi(i, j - 1, k), phi(i, j, k - 1),
                    phi(i, j, k));
      }
    }
  }

  // f, f, b
  for (int i = 1; i < phi.sx; i++) {
    for (int j = 1; j < phi.sy; j++) {
      for (int k = phi.sz - 2; k >= 0; k--) {
        if (marker(i, j, k) != FLUID_CELL)
          solve_phi(phi(i - 1, j, k), phi(i, j - 1, k), phi(i, j, k + 1),
                    phi(i, j, k));
      }
    }
  }

  // f, b, f
  for (int i = 1; i < phi.sx; i++) {
    for (int j = phi.sy - 2; j >= 0; j--) {
      for (int k = 1; k < phi.sz; k++) {
        if (marker(i, j, k) != FLUID_CELL)
          solve_phi(phi(i - 1, j, k), phi(i, j + 1, k), phi(i, j, k - 1),
                    phi(i, j, k));
      }
    }
  }

  // f, b, b
  for (int i = 1; i < phi.sx; i++) {
    for (int j = phi.sy - 2; j >= 0; j--) {
      for (int k = phi.sz - 2; k >= 0; k--) {
        if (marker(i, j, k) != FLUID_CELL)
          solve_phi(phi(i - 1, j, k), phi(i, j + 1, k), phi(i, j, k + 1),
                    phi(i, j, k));
      }
    }
  }
  //-----------

  // b, f, f
  for (int i = phi.sx - 2; i >= 0; i--) {
    for (int j = 1; j < phi.sy; j++) {
      for (int k = 1; k < phi.sz; k++) {
        if (marker(i, j, k) != FLUID_CELL)
          solve_phi(phi(i + 1, j, k), phi(i, j - 1, k), phi(i, j, k - 1),
                    phi(i, j, k));
      }
    }
  }

  // b, f, b
  for (int i = phi.sx - 2; i >= 0; i--) {
    for (int j = 1; j < phi.sy; j++) {
      for (int k = phi.sz - 2; k >= 0; k--) {
        if (marker(i, j, k) != FLUID_CELL)
          solve_phi(phi(i + 1, j, k), phi(i, j - 1, k), phi(i, j, k + 1),
                    phi(i, j, k));
      }
    }
  }

  // b, b, f
  for (int i = phi.sx - 2; i >= 0; i--) {
    for (int j = phi.sy - 2; j >= 0; j--) {
      for (int k = 1; k < phi.sz; k++) {
        if (marker(i, j, k) != FLUID_CELL)
          solve_phi(phi(i + 1, j, k), phi(i, j + 1, k), phi(i, j, k - 1),
                    phi(i, j, k));
      }
    }
  }

  // b, b, b
  for (int i = phi.sx - 2; i >= 0; i--) {
    for (int j = phi.sy - 2; j >= 0; j--) {
      for (int k = phi.sz - 2; k >= 0; k--) {
        if (marker(i, j, k) != FLUID_CELL)
          solve_phi(phi(i + 1, j, k), phi(i, j + 1, k), phi(i, j, k + 1),
                    phi(i, j, k));
      }
    }
  }
}

void Grid::solve_phi(float p, float q, float r, float &c) {
  float md = std::fmin(std::fmin(p, q), r) + 1;
  // FIXME improve sweeping algorithm
  // if (d > std::fmax(p,q)) {
  //   d =
  // }
  if (md < c)
    c = md;
}