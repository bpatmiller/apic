#pragma once

#include "array3.h"

class Grid {
public:
  float gravity = -9.8;

  float lx, ly, lz; // total length of grid
  float nx, ny, nz; // number of cells per dimension
  float h;          // size of each cell

  Array3f u, v, w;    // velocities
  Array3f du, dv, dw; // saved velocities (flip)
  Array3f count;      // keep track of how many particles
                      // are near each grid node
  Array3i marker;     // liquid, air, solid
  Array3f phi;        // used for velocity extrapolation
  Array3f pressure;   // self explanatory
  Array3v3 poission;  //
  Array3f precon;     //
  Array3f m;          // masses
  Array3f r, z, s;    //

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
    count.init(nx + 1, ny + 1, nz + 1);
    pressure.init(nx, ny, nz);

    marker.init(nx, ny, nz);
    phi.init(nx, ny, nz);
    poission.init(nx, ny, nz);
    precon.init(nx, ny, nz);
    m.init(nx, ny, nz);
    r.init(nx, ny, nz);
    z.init(nx, ny, nz);
    s.init(nx, ny, nz);
  }

  float CFL();
  void add_gravity(float dt);
  void compute_phi();
  void sweep_phi();
  void extend_velocity();
  void enforce_boundary();
  void project();
};