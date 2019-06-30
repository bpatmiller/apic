#pragma once

#include "array3.h"

#include <eigen3/Eigen/SparseCore>

#define AIR_CELL 0
#define FLUID_CELL 1
#define SOLID_CELL 2

#define U_OFFSET glm::vec3(0.5f, 0, 0)
#define V_OFFSET glm::vec3(0, 0.5f, 0)
#define W_OFFSET glm::vec3(0, 0, 0.5f)
#define CENTER_OFFSET glm::vec3(0.5f, 0.5f, 0.5f)

class Grid {
public:
  float gravity = -9.8f; // force of gravity

  float lx, ly, lz;     // total length of grid
  float nx, ny, nz;     // number of cells per dimension
  float h;              // size of each cell
  float density = 8.0f; //

  Array3f u, v, w;       // velocities sampled at cell faces
  Array3f u_w, v_w, w_w; // keep track of the total weight of particles
  Array3f du, dv, dw;    // saved velocities for flip
  Array3i marker;        // designates air, fluid, solid
  Array3f phi;           // signed distances from fluid
  Array3f solid_phi;     // solid signed distance function
  Array3d pressure;      // pressure at each grid cell
  Array3d r;             // divergence at each grid cell
  Array3i fl_index;      // gives each fluid cell an index (used for poission
                         // construction)
  Array3i pc;            // count used for reseeding

  // viscosity data
  Array3f u_vol, v_vol, w_vol, c_vol, n_vol;
  Array3f viscosity;
  Array3f visc_count;

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
    u_w.init(nx + 1, ny, nz);
    v_w.init(nx, ny + 1, nz);
    w_w.init(nx, ny, nz + 1);
    du.init(nx + 1, ny, nz);
    dv.init(nx, ny + 1, nz);
    dw.init(nx, ny, nz + 1);

    marker.init(nx, ny, nz);
    phi.init(nx, ny, nz);
    solid_phi.init(nx, ny, nz);
    pressure.init(nx, ny, nz);
    r.init(nx, ny, nz);
    fl_index.init(nx, ny, nz);
    pc.init(nx, ny, nz);

    // visc
    viscosity.init(nx, ny, nz);
    visc_count.init(nx, ny, nz);
    u_vol.init(nx + 1, ny, nz);
    v_vol.init(nx, ny + 1, nz);
    w_vol.init(nx, ny, nz + 1);
    c_vol.init(nx, ny, nz);
    n_vol.init(nx + 1, ny + 1, nz + 1);
  }

  void reset() {
    u.clear();
    v.clear();
    w.clear();
    du.clear();
    dv.clear();
    dw.clear();
    u_w.clear();
    v_w.clear();
    w_w.clear();

    for (int i = 0; i < marker.size; i++) {
      if (marker.data[i] != SOLID_CELL)
        marker.data[i] = 0;
    }
    phi.clear();
    solid_phi.clear();
    pressure.clear();
    r.clear();
    fl_index.clear();
    pc.clear();
  }

  // get max allowable timestep
  float CFL();
  // add gravity
  void add_gravity(float dt);
  // compute signed distance function / level set
  void compute_phi();
  void sweep_phi();
  void solve_phi(float p, float q, float r, float &c);
  // extrapolate velocity
  void extend_velocity();
  void sweep_velocity();
  void sweep_velocity_boundary(Array3f &arr);
  void sweep_u(int i0, int i1, int j0, int j1, int k0, int k1);
  void sweep_v(int i0, int i1, int j0, int j1, int k0, int k1);
  void sweep_w(int i0, int i1, int j0, int j1, int k0, int k1);
  // enforce boundary conditions
  void enforce_boundary();
  // save velocity field (FLIP)
  void save_velocity();
  // pressure solve
  void project(float dt);
  void compute_divergence();
  void solve_pressure();
  void add_pressure_gradient(float dt);
  void solve_x_helper(std::vector<Eigen::Triplet<double>> &tl, double &aii,
                      float dt, int i, int j, int k, int i1, int j1, int k1);
  void solve_x(float dt);
  // apply viscosity
  void apply_viscosity(float dt);
  void compute_viscosity_weights(float dt);
  void solve_viscosity(float dt);
  void compute_volume_fractions(Array3f &level, Array3f &arr, glm::vec3 offset,
                                int substeps);
};