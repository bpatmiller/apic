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

void Grid::extend_velocity() {
  for (int i = 0; i < 4; i++) {
    sweep_velocity();
  }
}

void Grid::sweep_velocity_boundary(Array3f &arr) {
  // top and bottom
  for (int i = 0; i < arr.sx; i++) {
    for (int k = 0; k < arr.sz; k++) {
      arr(i, 0, k) = arr(i, 1, k);
      arr(i, arr.sy - 1, k) = arr(i, arr.sy - 2, k);
    }
  }
  // left and right
  for (int j = 0; j < arr.sy; j++) {
    for (int k = 0; k < arr.sz; k++) {
      arr(0, j, k) = arr(1, j, k);
      arr(arr.sx - 1, j, k) = arr(arr.sx - 2, j, k);
    }
  }
  // front and back
  for (int i = 0; i < arr.sx; i++) {
    for (int j = 0; j < arr.sy; j++) {
      arr(i, j, 0) = arr(i, j, 1);
      arr(i, j, arr.sz - 1) = arr(i, j, arr.sz - 2);
    }
  }
}

void Grid::sweep_velocity() {
  // U --------------------------------
  sweep_u(1, u.sx - 1, 1, u.sy - 1, 1, u.sz - 1);
  sweep_u(1, u.sx - 1, 1, u.sy - 1, u.sz - 2, 0);
  sweep_u(1, u.sx - 1, u.sy - 2, 0, 1, u.sz - 1);
  sweep_u(1, u.sx - 1, u.sy - 2, 0, u.sz - 2, 0);
  sweep_u(u.sx - 2, 0, 1, u.sy - 1, 1, u.sz - 1);
  sweep_u(u.sx - 2, 0, 1, u.sy - 1, u.sz - 2, 0);
  sweep_u(u.sx - 2, 0, u.sy - 2, 0, 1, u.sz - 1);
  sweep_u(u.sx - 2, 0, u.sy - 2, 0, u.sz - 2, 0);
  // set boundary cells
  sweep_velocity_boundary(u);

  // V --------------------------------
  sweep_v(1, v.sx - 1, 1, v.sy - 1, 1, v.sz - 1);
  sweep_v(1, v.sx - 1, 1, v.sy - 1, v.sz - 2, 0);
  sweep_v(1, v.sx - 1, v.sy - 2, 0, 1, v.sz - 1);
  sweep_v(1, v.sx - 1, v.sy - 2, 0, v.sz - 2, 0);
  sweep_v(v.sx - 2, 0, 1, v.sy - 1, 1, v.sz - 1);
  sweep_v(v.sx - 2, 0, 1, v.sy - 1, v.sz - 2, 0);
  sweep_v(v.sx - 2, 0, v.sy - 2, 0, 1, v.sz - 1);
  sweep_v(v.sx - 2, 0, v.sy - 2, 0, v.sz - 2, 0);
  // set boundary cells
  sweep_velocity_boundary(v);

  // W --------------------------------
  sweep_w(1, w.sx - 1, 1, w.sy - 1, 1, w.sz - 1);
  sweep_w(1, w.sx - 1, 1, w.sy - 1, w.sz - 2, 0);
  sweep_w(1, w.sx - 1, w.sy - 2, 0, 1, w.sz - 1);
  sweep_w(1, w.sx - 1, w.sy - 2, 0, w.sz - 2, 0);
  sweep_w(w.sx - 2, 0, 1, w.sy - 1, 1, w.sz - 1);
  sweep_w(w.sx - 2, 0, 1, w.sy - 1, w.sz - 2, 0);
  sweep_w(w.sx - 2, 0, w.sy - 2, 0, 1, w.sz - 1);
  sweep_w(w.sx - 2, 0, w.sy - 2, 0, w.sz - 2, 0);
  // set boundary cells
  sweep_velocity_boundary(w);
}
void Grid::sweep_u(int i0, int i1, int j0, int j1, int k0, int k1) {
  //  int di=(i0<i1) ? 1 : -1;
  //  int dj=(j0<j1) ? 1 : -1;
  //  int dk=(k0<k1) ? 1 : -1;
}
void Grid::sweep_v(int i0, int i1, int j0, int j1, int k0, int k1) {}
void Grid::sweep_w(int i0, int i1, int j0, int j1, int k0, int k1) {}

void Grid::enforce_boundary() {
  // TODO
  // just zero out velocity components normal to the 6 cube faces
}
void Grid::project() {
  // TODO yeet lol
}

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

void Grid::compute_divergence() {
  r.clear();
  for (int i = 0; i < r.sx; i++) {
    for (int j = 0; j < r.sy; j++) {
      for (int k = 0; k < r.sz; k++) {
        if (marker(i, j, k) == FLUID_CELL)
          r(i, j, k) = u(i + 1, j, k) - u(i, j, k) + v(i, j + 1, k) -
                       v(i, j, k) + w(i, j, k + 1) - w(i, j, k);
      }
    }
  }
}