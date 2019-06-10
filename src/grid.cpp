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
  return std::max(1e-6f, h / std::sqrt(maxsq));
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
          float dr = 0.5 * (phi(i-1, j , k) + phi(i, j, k) -
                            phi(i-1, j , k - dk) - phi(i, j, k - dk));
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
          float dr = 0.5 * (phi(i-1, j - 1, k) + phi(i, j, k) -
                            phi(i-1, j - 1, k - dk) - phi(i, j, k - dk));
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
          float dq = 0.5 * (phi(i, j-1, k) + phi(i, j, k) -
                            phi(i, j - dj, k-dk) - phi(i, j - 1, k-dk));
          if (dq < 0)
            continue;
          // avg x-dir phi change
          float dp = 0.5 * (phi(i-1, j , k) + phi(i, j, k) -
                            phi(i - 1, j - 1, k-dk) - phi(i - 1, j, k-dk));
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
      v(i,0,k) = 0;
      v(i, 1, k) = 0;
      v(i, v.sy - 1, k) = 0;
      v(i, v.sy - 2, k) = 0;

    }
  }
  // left and right
  for (int j = 0; j < u.sy; j++) {
    for (int k = 0; k < u.sz; k++) {
      u(0,j,k)=0;
      u(1, j, k) = 0;
      u(u.sx - 1, j, k) = 0;
      u(u.sx - 2, j, k) = 0;

    }
  }
  // front and back
  for (int i = 0; i < w.sx; i++) {
    for (int j = 0; j < w.sy; j++) {
      w(i,j,0)=0;
      w(i, j, 1) = 0;
      w(i, j, w.sz - 1) = 0;
        w(i, j, w.sz - 2) = 0;

    }
  }
}

void Grid::project(float dt) {
  compute_divergence();
  form_poisson(dt);
  form_preconditioner();
  solve_pressure();
  add_pressure_gradient(dt);
}

void Grid::form_poisson(float dt) {
  poisson.clear();
  double scale = 1.0;//dt / (density * h * h);
  // float scale = 1.0;
  for (int i = 1; i < poisson.sx; i++) {
    for (int j = 1; j < poisson.sy; j++) {
      for (int k = 1; k < poisson.sz; k++) {
        if (marker(i, j, k) == FLUID_CELL) {
          // plus and minus x
          if (marker(i - 1, j, k) == FLUID_CELL) {
            poisson(i, j, k)[0] += scale;
          }
          if (marker(i + 1, j, k) == FLUID_CELL) {
            poisson(i, j, k)[0] += scale;
            poisson(i, j, k)[1] -= scale;
          } else if (marker(i + 1, j, k) == AIR_CELL) {
            poisson(i, j, k)[0] += scale;
          }
          // plus and minus y
          if (marker(i, j - 1, k) == FLUID_CELL) {
            poisson(i, j, k)[0] += scale;
          }
          if (marker(i, j + 1, k) == FLUID_CELL) {
            poisson(i, j, k)[0] += scale;
            poisson(i, j, k)[2] -= scale;
          } else if (marker(i, j + 1, k) == AIR_CELL) {
            poisson(i, j, k)[0] += scale;
          }
          // plus and minus z
          if (marker(i, j, k - 1) == FLUID_CELL) {
            poisson(i, j, k)[0] += 1.0;
          }
          if (marker(i, j, k + 1) == FLUID_CELL) {
            poisson(i, j, k)[0] += 1.0;
            poisson(i, j, k)[3] -= 1.0;
          } else if (marker(i, j, k + 1) == AIR_CELL) {
            poisson(i, j, k)[0] += 1.0;
          }
        }
      }
    }
  }
}
void Grid::apply_poisson(Array3d &x, Array3d &y) {
  y.clear();
  for (int i = 1; i < poisson.sx - 1; i++) {
    for (int j = 1; j < poisson.sy - 1; j++) {
      for (int k = 1; k < poisson.sz - 1; k++) {
        if (marker(i, j, k) == FLUID_CELL) {
          y(i, j, k) = poisson(i, j, k)[0] * x(i, j, k) +
                       poisson(i - 1, j, k)[1] * x(i - 1, j, k) +
                       poisson(i, j, k)[1] * x(i + 1, j, k) +
                       poisson(i, j - 1, k)[2] * x(i, j - 1, k) +
                       poisson(i, j, k)[2] * x(i, j + 1, k) +
                       poisson(i, j, k - 1)[3] * x(i, j, k - 1) +
                       poisson(i, j, k)[3] * x(i, j, k + 1);
        }
      }
    }
  }
}

void Grid::form_preconditioner() {
  precon.clear();
  double error_param = 0.97;
  double tuning = 0.25;
  for (int i = 1; i < precon.sx - 1; i++) {
    for (int j = 1; j < precon.sy - 1; j++) {
      for (int k = 1; k < precon.sz - 1; k++) {
        double d =
            poisson(i, j, k)[0] -
            sqr(poisson(i - 1, j, k)[1] * precon(i - 1, j, k)) -
            sqr(poisson(i, j - 1, k)[2] * precon(i, j - 1, k)) -
            sqr(poisson(i, j, k - 1)[3] * precon(i, j, k - 1)) -
            error_param *
                (poisson(i - 1, j, k)[1] *
                     (poisson(i - 1, j, k)[2] + poisson(i - 1, j, k)[3]) *
                     sqr(precon(i - 1, j, k)) +
                 poisson(i, j - 1, k)[2] *
                     (poisson(i, j - 1, k)[1] + poisson(i, j - 1, k)[3]) *
                     sqr(precon(i, j - 1, k)) +
                 poisson(i, j, k - 1)[3] *
                     (poisson(i, j, k - 1)[1] + poisson(i, j, k - 1)[2]) *
                     sqr(precon(i, j, k - 1)));

        if (d < tuning * poisson(i, j, k)[0])
          d = poisson(i, j, k)[0];
        if (d == 0)
          d = 1e-6;
        precon(i, j, k) = 1.0 / std::sqrt(d);
      }
    }
  }
}

void Grid::apply_preconditioner(Array3d &x, Array3d &y, Array3d &m) {
  // Lq=r
  m.clear();
  for (int i = 1; i < m.sx - 1; i++) {
    for (int j = 1; j < m.sy - 1; j++) {
      for (int k = 1; k < m.sz - 1; k++) {
        if (marker(i, j, k) == FLUID_CELL) {
          double t =
              x(i, j, k) -
              poisson(i - 1, j, k)[1] * precon(i - 1, j, k) * m(i - 1, j, k) -
              poisson(i, j - 1, k)[2] * precon(i, j - 1, k) * m(i, j - 1, k) -
              poisson(i, j, k - 1)[3] * precon(i, j, k - 1) * m(i, j, k - 1);
          m(i, j, k) = t * precon(i, j, k);
        }
      }
    }
  }

  // L'z=q
  y.clear();
  for (int i = x.sx - 2; i > 0; i--) {
    for (int j = x.sy - 2; j > 0; j--) {
      for (int k = x.sz - 2; k > 0; k--) {
        if (marker(i, j, k) == FLUID_CELL) {
          double t = m(i, j, k) -
                     poisson(i, j, k)[1] * precon(i, j, k) * y(i + 1, j, k) -
                     poisson(i, j, k)[2] * precon(i, j, k) * y(i, j + 1, k) -
                     poisson(i, j, k)[3] * precon(i, j, k) * y(i, j, k + 1);
          y(i, j, k) = t * precon(i, j, k);
        }
      }
    }
  }
}

void Grid::solve_pressure() {
  int max_its = 500;
  double tol = 1e-7 * r.infnorm();
  pressure.clear();
  // if no divergence
  if (r.infnorm() == 0)
    return;
  apply_preconditioner(r, z, m);
  z.copy(s);
  double sigma = z.dot(r);
  if (sigma == 0)
    return;
  for (int i = 0; i < max_its; i++) {
    apply_poisson(s, z);
    double a = sigma / s.dot(z);
    pressure.inc(a, s);
    r.inc(-a, z);
    if (r.infnorm() <= tol) {
      // std::cout << "pressure converged with tol " << r.infnorm() << " and "
      // << i
      //           << " iterations" << std::endl;
      return;
    }
    apply_preconditioner(r, z, m);
    double sigma_n = z.dot(r);
    double b = sigma_n / sigma;
    s.scale_inc(b, z);
    sigma = sigma_n;
  }
  std::cout << "error, pressure did not converge || r=" << r.infnorm()
            << std::endl;
}

// add the new pressure gradient to the velocity field
void Grid::add_pressure_gradient(float dt) {
  double scale = 1.0;//-dt / (density * h);
  // u
  for (int i = 2; i < u.sx - 2; i++) {
    for (int j = 1; j < u.sy - 1; j++) {
      for (int k = 1; k < u.sz - 1; k++) {
        if (marker(i - 1, j, k) == SOLID_CELL || marker(i, j, k) == SOLID_CELL)
          continue;
        if (marker(i - 1, j, k) == FLUID_CELL ||
            marker(i, j, k) == FLUID_CELL) {
          u(i, j, k) += scale * (pressure(i, j, k) - pressure(i - 1, j, k));

        }
      }
    }
  }

  // v
  for (int i = 1; i < v.sx - 1; i++) {
    for (int j = 2; j < v.sy - 2; j++) {
      for (int k = 1; k < v.sz - 1; k++) {
        if (marker(i, j - 1, k) == SOLID_CELL || marker(i, j, k) == SOLID_CELL)
          continue;
        if (marker(i, j - 1, k) == FLUID_CELL ||
            marker(i, j, k) == FLUID_CELL) {
          v(i, j, k) += scale * (pressure(i, j, k) - pressure(i, j - 1, k));

        }
      }
    }
  }

  // w
  for (int i = 1; i < w.sx - 1; i++) {
    for (int j = 1; j < w.sy - 1; j++) {
      for (int k = 2; k < w.sz - 2; k++) {
        if (marker(i, j, k - 1) == SOLID_CELL || marker(i, j, k) == SOLID_CELL)
          continue;
        if (marker(i, j, k - 1) == FLUID_CELL ||
            marker(i, j, k) == FLUID_CELL) {
          w(i, j, k) += scale * (pressure(i, j, k) - pressure(i, j, k - 1));

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
  float md = std::fmin(std::fmin(p, q), r) + 1;
  // FIXME improve sweeping algorithm
  // if (md > std::fmax(p,q)) {
  //   md = (p+q+sqrt(2-sqr(p-q)))/2;
  // }
  if (md < c)
    c = md;
}

void Grid::compute_divergence() {
  double scale = 1.0;//-1.0 / h;
  r.clear();
  for (int i = 0; i < r.sx; i++) {
    for (int j = 0; j < r.sy; j++) {
      for (int k = 0; k < r.sz; k++) {
        if (marker(i, j, k) == FLUID_CELL)
          r(i, j, k) = scale * (u(i + 1, j, k) - u(i, j, k) + v(i, j + 1, k) -
                                v(i, j, k) + w(i, j, k + 1) - w(i, j, k));
      }
    }
  }
}