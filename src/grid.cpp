#include "grid.h"
#include "Particle.h"
#include <algorithm>
#include <eigen3/Eigen/IterativeLinearSolvers>
#include <eigen3/Eigen/SparseCholesky>
#include <iostream>

static inline int ijk_to_index(int i, int j, int k, Array3f &arr) {
  return i + (arr.sx * j) + (arr.sx * arr.sy * k);
}

static inline double sqr(double d) { return std::pow(d, 2.0); }
static inline float sqr(float f) { return std::pow(f, 2.0f); }

// ensure no particle moves more than grid.h
float Grid::CFL() {
  float maxsq = sqr(u.infnorm()) + sqr(w.infnorm()) + sqr(v.infnorm());
  maxsq = std::max(maxsq, 1e-16f);
  return h / std::sqrt(maxsq);
}

void Grid::add_gravity(float dt) {
  float g = dt * gravity;
  for (int i = 0; i < v.size; i++) {
    v.data[i] += g;
  }
}

void Grid::extend_velocity() {
  for (int i = 0; i < 8; i++) {
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
  int di = (i0 < i1) ? 1 : -1;
  int dj = (j0 < j1) ? 1 : -1;
  int dk = (k0 < k1) ? 1 : -1;

  float weight;

  for (int i = i0; i != i1; i += di) {
    for (int j = j0; j != j1; j += dj) {
      for (int k = k0; k != k1; k += dk) {
        if (marker(i - 1, j, k) == AIR_CELL && marker(i, j, k) == AIR_CELL) {
          float dp = di * (phi(i, j, k) - phi(i - 1, j, k));
          if (dp < 0)
            continue;
          // avg y-dir phi change
          float dq = 0.5 * (phi(i - 1, j, k) + phi(i, j, k) -
                            phi(i - 1, j - dj, k) - phi(i, j - dj, k));
          if (dq < 0)
            continue;
          // avg z-dir phi chhttps://www.youtube.com/watch?v=HcO-NPtj5BIange
          float dr = 0.5 * (phi(i - 1, j, k) + phi(i, j, k) -
                            phi(i - 1, j, k - dk) - phi(i, j, k - dk));
          if (dr < 0)
            continue;

          // weighted sum of other velocities
          if (dp + dq + dr == 0) {
            weight = 1.0f / 3.0f;
            u(i, j, k) =
                weight * (u(i - di, j, k) + u(i, j - dj, k) + u(i, j, k - dk));
          } else {
            weight = 1.0f / (dp + dq + dr);
            u(i, j, k) = dp * weight * u(i - di, j, k) +
                         dq * weight * u(i, j - dj, k) +
                         dr * weight * u(i, j, k - dk);
          }
        }
      }
    }
  }
}
void Grid::sweep_v(int i0, int i1, int j0, int j1, int k0, int k1) {
  int di = (i0 < i1) ? 1 : -1;
  int dj = (j0 < j1) ? 1 : -1;
  int dk = (k0 < k1) ? 1 : -1;

  float weight;

  for (int i = i0; i != i1; i += di) {
    for (int j = j0; j != j1; j += dj) {
      for (int k = k0; k != k1; k += dk) {
        if (marker(i, j - 1, k) == AIR_CELL && marker(i, j, k) == AIR_CELL) {
          float dq = dj * (phi(i, j, k) - phi(i, j - 1, k));
          if (dq < 0)
            continue;
          // avg x-dir phi change
          float dp = 0.5 * (phi(i, j - 1, k) + phi(i, j, k) -
                            phi(i - di, j - 1, k) - phi(i - di, j, k));
          if (dp < 0)
            continue;
          // avg z-dir phi change
          float dr = 0.5 * (phi(i - 1, j - 1, k) + phi(i, j, k) -
                            phi(i - 1, j - 1, k - dk) - phi(i, j, k - dk));
          if (dr < 0)
            continue;

          // weighted sum of other velocities
          if (dp + dq + dr == 0) {
            weight = 1.0f / 3.0f;
            v(i, j, k) =
                weight * (v(i - di, j, k) + v(i, j - dj, k) + v(i, j, k - dk));
          } else {
            weight = 1.0f / (dp + dq + dr);
            v(i, j, k) = dp * weight * v(i - di, j, k) +
                         dq * weight * v(i, j - dj, k) +
                         dr * weight * v(i, j, k - dk);
          }
        }
      }
    }
  }
}

void Grid::sweep_w(int i0, int i1, int j0, int j1, int k0, int k1) {
  int di = (i0 < i1) ? 1 : -1;
  int dj = (j0 < j1) ? 1 : -1;
  int dk = (k0 < k1) ? 1 : -1;

  float weight;

  for (int i = i0; i != i1; i += di) {
    for (int j = j0; j != j1; j += dj) {
      for (int k = k0; k != k1; k += dk) {
        if (marker(i, j, k - 1) == AIR_CELL && marker(i, j, k) == AIR_CELL) {
          float dr = dk * (phi(i, j, k) - phi(i, j, k - 1));
          if (dr < 0)
            continue;
          // avg y-dir phi change
          float dq = 0.5 * (phi(i, j - 1, k) + phi(i, j, k) -
                            phi(i, j - dj, k - dk) - phi(i, j - 1, k - dk));
          if (dq < 0)
            continue;
          // avg x-dir phi change
          float dp = 0.5 * (phi(i - 1, j, k) + phi(i, j, k) -
                            phi(i - 1, j - 1, k - dk) - phi(i - 1, j, k - dk));
          if (dp < 0)
            continue;

          // weighted sum of other velocities
          if (dp + dq + dr == 0) {
            weight = 1.0f / 3.0f;
            w(i, j, k) =
                weight * (w(i - di, j, k) + w(i, j - dj, k) + w(i, j, k - dk));
          } else {
            weight = 1.0f / (dp + dq + dr);
            w(i, j, k) = dp * weight * w(i - di, j, k) +
                         dq * weight * w(i, j - dj, k) +
                         dr * weight * w(i, j, k - dk);
          }
        }
      }
    }
  }
}

void Grid::enforce_boundary() {
  // just zero out velocity components normal to the 6 cube faces
  // top and bottom
  for (int i = 0; i < v.sx; i++) {
    for (int k = 0; k < v.sz; k++) {
      v(i, 0, k) = 0;
      v(i, 1, k) = 0;
      v(i, v.sy - 1, k) = 0;
      v(i, v.sy - 2, k) = 0;
    }
  }
  // left and right
  for (int j = 0; j < u.sy; j++) {
    for (int k = 0; k < u.sz; k++) {
      u(0, j, k) = 0;
      u(1, j, k) = 0;
      u(u.sx - 1, j, k) = 0;
      u(u.sx - 2, j, k) = 0;
    }
  }
  // front and back
  for (int i = 0; i < w.sx; i++) {
    for (int j = 0; j < w.sy; j++) {
      w(i, j, 0) = 0;
      w(i, j, 1) = 0;
      w(i, j, w.sz - 1) = 0;
      w(i, j, w.sz - 2) = 0;
    }
  }
}

void Grid::project(float dt) {
  compute_divergence();
  solve_x(dt);
  add_pressure_gradient(dt);
}

void Grid::solve_x_helper(std::vector<Eigen::Triplet<double>> &tl, double &aii,
                          float dt, int i, int j, int k, int i1, int j1,
                          int k1) {
  if (marker(i1, j1, k1) != SOLID_CELL) {
    aii += 1;
    if (marker(i1, j1, k1) == FLUID_CELL) {
      double scale = dt / (density * h * h);
      tl.push_back(Eigen::Triplet<double>(fl_index(i, j, k),
                                          fl_index(i1, j1, k1), -scale));
    }
  }
}

void Grid::solve_x(float dt) {
  int nf = 0; // total number of fluid cells

  // assign an index to each fluid cell
  fl_index.clear();
  for (int i = 0; i < fl_index.sx; i++) {
    for (int j = 0; j < fl_index.sy; j++) {
      for (int k = 0; k < fl_index.sz; k++) {
        if (marker(i, j, k) == FLUID_CELL) {
          fl_index(i, j, k) = nf;
          nf += 1;
        }
      }
    }
  }

  // populate triplets
  std::vector<Eigen::Triplet<double>> tripletList;
  tripletList.reserve(nf * 7);
  double scale = dt / (density * h * h);
  for (int i = 0; i < r.sx; i++) {
    for (int j = 0; j < r.sy; j++) {
      for (int k = 0; k < r.sz; k++) {
        if (marker(i, j, k) == FLUID_CELL) {
          int index = fl_index(i, j, k);
          double aii = 0; // negative sum of nonsolid nbrs
          solve_x_helper(tripletList, aii, dt, i, j, k, i + 1, j, k);
          solve_x_helper(tripletList, aii, dt, i, j, k, i - 1, j, k);
          solve_x_helper(tripletList, aii, dt, i, j, k, i, j + 1, k);
          solve_x_helper(tripletList, aii, dt, i, j, k, i, j - 1, k);
          solve_x_helper(tripletList, aii, dt, i, j, k, i, j, k + 1);
          solve_x_helper(tripletList, aii, dt, i, j, k, i, j, k - 1);
          tripletList.push_back(
              Eigen::Triplet<double>(index, index, aii * scale));
        }
      }
    }
  }
  // construct A from triplets
  Eigen::SparseMatrix<double> A(nf, nf);
  A.setFromTriplets(tripletList.begin(), tripletList.end());

  // construct b
  Eigen::VectorXd b(nf);
  for (int i = 0; i < r.sx; i++) {
    for (int j = 0; j < r.sy; j++) {
      for (int k = 0; k < r.sz; k++) {
        if (marker(i, j, k) == FLUID_CELL) {
          b(fl_index(i, j, k)) = (-1.0 / h) * r(i, j, k);
        }
      }
    }
  }

  // solve Ax=b
  Eigen::VectorXd x(nf);
  Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> solver;
  solver.compute(A);
  x = solver.solve(b);

  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("pressure not solved");
  }

  // set pressure
  pressure.clear();
  for (int i = 0; i < fl_index.sx; i++) {
    for (int j = 0; j < fl_index.sy; j++) {
      for (int k = 0; k < fl_index.sz; k++) {
        if (marker(i, j, k) == FLUID_CELL) {
          pressure(i, j, k) = x(fl_index(i, j, k));
        }
      }
    }
  }
}

// add the new pressure gradient to the velocity field
void Grid::add_pressure_gradient(float dt) {
  double scale = -dt / (density * h);
  // u
  for (int i = 2; i < u.sx - 2; i++) {
    for (int j = 1; j < u.sy - 1; j++) {
      for (int k = 1; k < u.sz - 1; k++) {

        if (marker(i - 1, j, k) == FLUID_CELL ||
            marker(i, j, k) == FLUID_CELL) {
          if (marker(i - 1, j, k) == SOLID_CELL ||
              marker(i, j, k) == SOLID_CELL) {
            continue;
          } else {
            u(i, j, k) += scale * (pressure(i, j, k) - pressure(i - 1, j, k));
          }
        }
      }
    }
  }

  // v
  for (int i = 1; i < v.sx - 1; i++) {
    for (int j = 2; j < v.sy - 2; j++) {
      for (int k = 1; k < v.sz - 1; k++) {
        if (marker(i, j - 1, k) == FLUID_CELL ||
            marker(i, j, k) == FLUID_CELL) {
          if (marker(i, j - 1, k) == SOLID_CELL ||
              marker(i, j, k) == SOLID_CELL) {
            continue;
          } else {
            v(i, j, k) += scale * (pressure(i, j, k) - pressure(i, j - 1, k));
          }
        }
      }
    }
  }

  // w
  for (int i = 1; i < w.sx - 1; i++) {
    for (int j = 1; j < w.sy - 1; j++) {
      for (int k = 2; k < w.sz - 2; k++) {
        if (marker(i, j, k - 1) == FLUID_CELL ||
            marker(i, j, k) == FLUID_CELL) {
          if (marker(i, j, k - 1) == SOLID_CELL ||
              marker(i, j, k) == SOLID_CELL) {
            continue;
          } else {
            w(i, j, k) += scale * (pressure(i, j, k) - pressure(i, j, k - 1));
          }
        }
      }
    }
  }
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

  for (int i = 0; i < 8; i++) {
    sweep_phi();
  }
}

// TODO abstract each direction into a function, as in sweeping velocity
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
  float pqmin = std::fmin(p, q);
  float pqmax = std::fmax(p, q);
  float min = std::fmin(r, pqmin);
  float max = std::fmax(r, pqmax);
  float mid = std::fmax(std::fmin(r, pqmax), pqmin);

  float d = min + 1;
  if (d > mid) {
    d = (min + mid + std::sqrt(2.0 - sqr(min - mid))) * 0.5;
    if (d > max) {
      d = (min + mid + max +
           std::sqrt(3.0 - 2.0 * (min * min - min * mid - min * max +
                                  mid * mid - mid * max + max * max))) /
          3.0;
    }
  }
  if (d < c)
    c = d;
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