#pragma once

#include "array3.h"

#include <eigen3/Eigen/SparseCore>

#define AIR_CELL 0
#define FLUID_CELL 1
#define SOLID_CELL 2

class Grid {
public:
  float gravity = -9.8;

  float lx, ly, lz;  // total length of grid
  float nx, ny, nz;  // number of cells per dimension
  float h;           // size of each cell
  float density = 1.0f; //

  Array3f u, v, w;    // velocities
  Array3f du, dv, dw; // saved velocities for flip
  Array3f count;      // keep track of how many particles
                      // are near each grid node
                      // (normalizing velocity field)
  Array3i marker;     // air, fluid, solid
  Array3f phi;        // signed distances
  Array3d pressure;   // self explanatory
  Array3f rho;        // density

  Array3v3 lap_pres;

  Array3v4 poisson; // Adiag, Ax, Ay, Az
  Array3d precon;   //
  Array3d m;        //
  Array3d r, z, s;  //

  // used for pressure solve
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd x, b;

  Grid() {}

  void init(float lx_, int nx_, int ny_, int nz_) {
    // number of cells
    nx = nx_;
    ny = ny_;
    nz = nz_;
    // cell size
    h = lx_ / static_cast<float>(nx);
    // grid side lengths
    lx = lx_;
    ly = h * static_cast<float>(ny);
    lz = h * static_cast<float>(nz);

    // velocities
    u.init(nx + 1, ny, nz);
    v.init(nx, ny + 1, nz);
    w.init(nx, ny, nz + 1);
    du.init(nx + 1, ny, nz);
    dv.init(nx, ny + 1, nz);
    dw.init(nx, ny, nz + 1);
    count.init(nx + 1, ny + 1, nz + 1);
    pressure.init(nx, ny, nz);
    rho.init(nx, ny, nz);

    lap_pres.init(nx, ny, nz);

    marker.init(nx, ny, nz);
    phi.init(nx, ny, nz);

    poisson.init(nx, ny, nz);
    precon.init(nx, ny, nz);
    m.init(nx, ny, nz);
    r.init(nx, ny, nz);
    z.init(nx, ny, nz);
    s.init(nx, ny, nz);
  }

  void reset() {
    u.clear();
    v.clear();
    w.clear();
    du.clear();
    dv.clear();
    dw.clear();
    count.clear();
    pressure.clear();
    rho.clear();

    lap_pres.clear();

    marker.clear();
    phi.clear();
    poisson.clear();
    precon.clear();
    m.clear();
    r.clear();
    z.clear();
    s.clear();
  }

  float CFL();
  void add_gravity(float dt);
  void compute_phi();
  void sweep_phi();
  void solve_phi(float p, float q, float r, float &c);
  void extend_velocity();
  void sweep_velocity();
  void sweep_velocity_boundary(Array3f &arr);
  void sweep_u(int i0, int i1, int j0, int j1, int k0, int k1);
  void sweep_v(int i0, int i1, int j0, int j1, int k0, int k1);
  void sweep_w(int i0, int i1, int j0, int j1, int k0, int k1);
  void enforce_boundary();

  void project(float dt);
  void compute_divergence();
  void form_poisson(float dt);
  void apply_poisson(Array3d &x, Array3d &y);
  void form_preconditioner();
  void apply_preconditioner(Array3d &x, Array3d &y, Array3d &m);
  void solve_pressure();
  void add_pressure_gradient(float dt);
};