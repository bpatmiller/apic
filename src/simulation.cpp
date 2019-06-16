#include "simulation.h"
#include <fstream>
#include <glm/gtc/random.hpp>
#include <glm/gtx/common.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>

#define U_OFFSET glm::vec3(0.5f, 0, 0)
#define V_OFFSET glm::vec3(0, 0.5f, 0)
#define W_OFFSET glm::vec3(0, 0, 0.5f)

#define EPS 0.001

// initialize "dam break" scenario
void Simulation::add_particle_box() {
  particles.clear();
  for (int x = grid.nx * 0.4; x < grid.nx * 0.6; x++) {
    for (int y = grid.ny * 0.2; y < grid.ny * 0.6; y++) {
      for (int z = grid.nz * 0.4; z < grid.nz * 0.6; z++) {
        // for each cell, add 8 new jittered particles
        float base_x = x * grid.h;
        float base_y = y * grid.h;
        float base_z = z * grid.h;
        for (int i = 0; i < 8; i++) {
          float jitter_x = glm::linearRand(0 + EPS, grid.h - EPS);
          float jitter_y = glm::linearRand(0 + EPS, grid.h - EPS);
          float jitter_z = glm::linearRand(0 + EPS, grid.h - EPS);
          // add particles
          particles.push_back(
              Particle(glm::vec3(base_x + jitter_x, base_y + jitter_y,
                                 base_z + jitter_z),
                       glm::vec3(0, -10.0f, 0)));
          // APIC vectors
          cx.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
          cy.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
          cz.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
        }
      }
    }
  }
}

void Simulation::save_velocities() {
  grid.u.copy(grid.du);
  grid.v.copy(grid.dv);
  grid.w.copy(grid.dw);
}

// given a position, return the trilinear interpolation
// of the velocity field at that position
glm::vec3 Simulation::trilerp_uvw(glm::vec3 p) {
  glm::ivec3 index;
  glm::vec3 coords;
  glm::vec3 result;
  // u
  position_to_grid(p, U_OFFSET, index, coords);
  result.x = grid.u.trilerp(index, coords);
  // v
  position_to_grid(p, V_OFFSET, index, coords);
  result.y = grid.v.trilerp(index, coords);
  // w
  position_to_grid(p, W_OFFSET, index, coords);
  result.z = grid.w.trilerp(index, coords);

  return result;
}

glm::vec3 Simulation::trilerp_dudvdw(glm::vec3 p) {
  glm::ivec3 index;
  glm::vec3 coords;
  glm::vec3 result;
  // u
  position_to_grid(p, U_OFFSET, index, coords);
  result.x = grid.du.trilerp(index, coords);
  // v
  position_to_grid(p, V_OFFSET, index, coords);
  result.y = grid.dv.trilerp(index, coords);
  // w
  position_to_grid(p, W_OFFSET, index, coords);
  result.z = grid.dw.trilerp(index, coords);

  return result;
}

// does the same as above, but rounds down for easier grid transfer
// use offset = glm::vec3(0) for values sampled at center
void Simulation::position_to_grid(glm::vec3 p, glm::vec3 offset,
                                  glm::ivec3 &index, glm::vec3 &coords) {
  float nx = (p.x / grid.h) - 0.5f + offset.x;
  float ny = (p.y / grid.h) - 0.5f + offset.y;
  float nz = (p.z / grid.h) - 0.5f + offset.z;

  int i = static_cast<int>(nx);
  int j = static_cast<int>(ny);
  int k = static_cast<int>(nz);

  // set index
  index = glm::ivec3(i, j, k);

  // check for out of bounds
  if (i < 0 || j < 0 || k < 0)
    std::cerr << "ERROR: invalid index || position: " << glm::to_string(p)
              << std::endl;

  float bx = nx - std::floor(nx);
  float by = ny - std::floor(ny);
  float bz = nz - std::floor(nz);

  // set barycentric coordinates
  coords = glm::vec3(bx, by, bz);
}

// arr = Array3f to be added to
// quantity = quantity to add in (e.g. x component of particle velocity)
// index = lowest index of grid nodes to be added to
// coords = coordinates relative to those lowest grid nodes (0.0-1.0)
template <class T>
void Simulation::grid_add_quantities(T &arr, float q, glm::ivec3 index,
                                     glm::vec3 coords) {
  int range = 1;
  int max_w = (2 * range) - 1;       // (2 * 1) - 1 = 1
  int scale = max_w * max_w * max_w; // 1 * 1 * 1 = 1
  float w;

  for (int i = 1 - range; i <= range; i++) { // {0,1}
    for (int j = 1 - range; j <= range; j++) {
      for (int k = 1 - range; k <= range; k++) {
        if (index.x + i < 0 || index.x + i >= arr.sx || index.y + j < 0 ||
            index.y + j >= arr.sy || index.z + k < 0 || index.z + k >= arr.sz)
          continue;
        w = (max_w - std::fabs(i - coords.x)) * // {1 - coords.x | coords.x}
            (max_w - std::fabs(j - coords.y)) * // {""}
            (max_w - std::fabs(k - coords.z)) * scale;       //{""}
        arr(index.x + i, index.y + j, index.z + k) += w * q; //{0,1}
        grid.count(index.x + i, index.y + j, index.z + k) += w;
      }
    }
  }
}

template <class T>
void Simulation::affine_set(T &accum, glm::vec3 c, glm::ivec3 index,
                            glm::vec3 coords) {
  float w;
  float coef;

  w = (1 - coords.x) * (1 - coords.y) * (1 - coords.z);
  coef = glm::dot(c, glm::vec3(-coords.x, -coords.y, -coords.z) * grid.h);
  accum(index.x, index.y, index.z) += w * coef;

  w = (1 - coords.x) * (1 - coords.y) * (coords.z);
  coef = glm::dot(c, glm::vec3(-coords.x, -coords.y, 1 - coords.z) * grid.h);
  accum(index.x, index.y, index.z + 1) += w * coef;

  w = (1 - coords.x) * (coords.y) * (1 - coords.z);
  coef = glm::dot(c, glm::vec3(-coords.x, 1 - coords.y, -coords.z) * grid.h);
  accum(index.x, index.y + 1, index.z) += w * coef;

  w = (1 - coords.x) * (coords.y) * (coords.z);
  coef = glm::dot(c, glm::vec3(-coords.x, 1 - coords.y, 1 - coords.z) * grid.h);
  accum(index.x, index.y + 1, index.z + 1) += w * coef;

  w = (coords.x) * (1 - coords.y) * (1 - coords.z);
  coef = glm::dot(c, glm::vec3(1 - coords.x, -coords.y, -coords.z) * grid.h);
  accum(index.x + 1, index.y, index.z) += w * coef;

  w = (coords.x) * (1 - coords.y) * (coords.z);
  coef = glm::dot(c, glm::vec3(1 - coords.x, -coords.y, 1 - coords.z) * grid.h);
  accum(index.x + 1, index.y, index.z + 1) += w * coef;

  w = (coords.x) * (coords.y) * (1 - coords.z);
  coef = glm::dot(c, glm::vec3(1 - coords.x, 1 - coords.y, -coords.z) * grid.h);
  accum(index.x + 1, index.y + 1, index.z) += w * coef;

  w = (coords.x) * (coords.y) * (coords.z);
  coef =
      glm::dot(c, glm::vec3(1 - coords.x, 1 - coords.y, 1 - coords.z) * grid.h);
  accum(index.x + 1, index.y + 1, index.z + 1) += w * coef;
}

// for each particle, trilinearly interpolate velocity
// to all grid points nearby
void Simulation::particles_to_grid() {
  // density
  grid.rho.clear();
  for (uint i = 0; i < particles.size(); i++) {
    Particle &p = particles[i];
    glm::ivec3 index;
    glm::vec3 coords;
    position_to_grid(p.position, glm::vec3(0.5f, 0.5f, 0.5f), index, coords);
    grid_add_quantities(grid.rho, 1.0, index, coords);
  }

  // u
  grid.u.clear();
  grid.count.clear();
  for (uint i = 0; i < particles.size(); i++) {
    Particle &p = particles[i];
    glm::ivec3 index;
    glm::vec3 coords;
    position_to_grid(p.position, U_OFFSET, index, coords);
    grid_add_quantities(grid.u, p.velocity.x, index, coords);
    if (mode == APIC_MODE) {
      affine_set(grid.u, cx[i], index, coords);
    }
  }
  // average velocities
  for (int i = 0; i < grid.u.sx; i++) {
    for (int j = 0; j < grid.u.sy; j++) {
      for (int k = 0; k < grid.u.sz; k++) {
        if (grid.count(i, j, k) != 0)
          grid.u(i, j, k) /= grid.count(i, j, k);
      }
    }
  }

  // v
  grid.v.clear();
  grid.count.clear();
  for (uint i = 0; i < particles.size(); i++) {
    Particle &p = particles[i];
    glm::ivec3 index;
    glm::vec3 coords;
    position_to_grid(p.position, V_OFFSET, index, coords);
    grid_add_quantities(grid.v, p.velocity.y, index, coords);
    if (mode == APIC_MODE) {
      affine_set(grid.v, cy[i], index, coords);
    }
  }
  // average velocities
  for (int i = 0; i < grid.v.sx; i++) {
    for (int j = 0; j < grid.v.sy; j++) {
      for (int k = 0; k < grid.v.sz; k++) {
        if (grid.count(i, j, k) != 0) {
          grid.v(i, j, k) /= grid.count(i, j, k);
        }
      }
    }
  }

  // w
  grid.w.clear();
  grid.count.clear();
  for (uint i = 0; i < particles.size(); i++) {
    Particle &p = particles[i];
    glm::ivec3 index;
    glm::vec3 coords;
    position_to_grid(p.position, W_OFFSET, index, coords);
    grid_add_quantities(grid.w, p.velocity.z, index, coords);
    if (mode == APIC_MODE) {
      affine_set(grid.w, cz[i], index, coords);
    }
  }
  // average velocities
  for (int i = 0; i < grid.w.sx; i++) {
    for (int j = 0; j < grid.w.sy; j++) {
      for (int k = 0; k < grid.w.sz; k++) {
        if (grid.count(i, j, k) != 0)
          grid.w(i, j, k) /= grid.count(i, j, k);
      }
    }
  }
}

// return gradient of weighted field
glm::vec3 Simulation::compute_C(Array3f &field, glm::ivec3 index,
                                glm::vec3 coords) {
  glm::vec3 c = glm::vec3(0.0f, 0.0f, 0.0f);
  glm::vec3 wv;
  float w;

  w = (1 - coords.x) * (1 - coords.y) * (1 - coords.z);
  wv = glm::vec3(-coords.x, -coords.y, -coords.z);
  c += w * wv * field(index.x, index.y, index.z);

  w = (1 - coords.x) * (1 - coords.y) * (coords.z);
  wv = glm::vec3(-coords.x, -coords.y, 1 - coords.z);
  c += w * wv * field(index.x, index.y, index.z + 1);

  w = (1 - coords.x) * (coords.y) * (1 - coords.z);
  wv = glm::vec3(-coords.x, 1 - coords.y, -coords.z);
  c += w * wv * field(index.x, index.y + 1, index.z);

  w = (1 - coords.x) * (coords.y) * (coords.z);
  wv = glm::vec3(-coords.x, 1 - coords.y, 1 - coords.z);
  c += w * wv * field(index.x, index.y + 1, index.z + 1);

  w = (coords.x) * (1 - coords.y) * (1 - coords.z);
  wv = glm::vec3(1 - coords.x, -coords.y, -coords.z);
  c += w * wv * field(index.x + 1, index.y, index.z);

  w = (coords.x) * (1 - coords.y) * (coords.z);
  wv = glm::vec3(1 - coords.x, -coords.y, 1 - coords.z);
  c += w * wv * field(index.x + 1, index.y, index.z + 1);

  w = (coords.x) * (coords.y) * (1 - coords.z);
  wv = glm::vec3(1 - coords.x, 1 - coords.y, -coords.z);
  c += w * wv * field(index.x + 1, index.y + 1, index.z);

  w = (coords.x) * (coords.y) * (coords.z);
  wv = glm::vec3(1 - coords.x, 1 - coords.y, 1 - coords.z);
  c += w * wv * field(index.x + 1, index.y + 1, index.z + 1);

  return c * grid.h;
}

void Simulation::grid_to_particles() {
  // pic flip velocity differences
  if (mode == PIC_FLIP_MODE) {
    for (int i = 0; i < grid.u.size; i++) {
      grid.du.data[i] = grid.u.data[i] - grid.du.data[i];
    }
    for (int i = 0; i < grid.v.size; i++) {
      grid.dv.data[i] = grid.v.data[i] - grid.dv.data[i];
    }
    for (int i = 0; i < grid.w.size; i++) {
      grid.dw.data[i] = grid.w.data[i] - grid.dw.data[i];
    }
  }

  for (uint i = 0; i < particles.size(); i++) {
    Particle &p = particles[i];
    if (mode == PIC_FLIP_MODE) {
      p.velocity = (1.0f - flip_blend) * trilerp_uvw(p.position) +
                   flip_blend * (p.velocity + trilerp_dudvdw(p.position));
    } else {
      p.velocity = trilerp_uvw(p.position);
      if (mode == APIC_MODE) {
        // transfer C to particles
        glm::ivec3 index;
        glm::vec3 coords;
        position_to_grid(p.position, U_OFFSET, index, coords);
        cx[i] = compute_C(grid.u, index, coords);
        position_to_grid(p.position, V_OFFSET, index, coords);
        cy[i] = compute_C(grid.v, index, coords);
        position_to_grid(p.position, W_OFFSET, index, coords);
        cz[i] = compute_C(grid.w, index, coords);
      }
    }
  }
}

void Simulation::advect(float dt) {
  glm::vec3 mid, gu;
  // boundaries
  float xmin = 1.01 * grid.h;
  float ymin = 1.01 * grid.h;
  float zmin = 1.01 * grid.h;
  float xmax = grid.lx - 1.01 * grid.h;
  float ymax = grid.ly - 1.01 * grid.h;
  float zmax = grid.lz - 1.01 * grid.h;
  for (uint i = 0; i < particles.size(); i++) {
    Particle &p = particles[i];
    // first stage of RK2
    gu = trilerp_uvw(p.position);
    mid = p.position + 0.5f * dt * gu;
    mid.x = glm::clamp(mid.x, xmin, xmax);
    mid.y = glm::clamp(mid.y, ymin, ymax);
    mid.z = glm::clamp(mid.z, zmin, zmax);
    // second stage
    gu = trilerp_uvw(mid);
    p.position += dt * gu;
    p.position.x = glm::clamp(p.position.x, xmin, xmax);
    p.position.y = glm::clamp(p.position.y, ymin, ymax);
    p.position.z = glm::clamp(p.position.z, zmin, zmax);
  }
}

void Simulation::mark_cells() {
  // start by setting cells as empty
  grid.marker.clear();

  // mark liquid cells
  glm::ivec3 index;
  glm::vec3 coords;
  for (Particle &p : particles) {
    position_to_grid(p.position, glm::vec3(0.5f, 0.5f, 0.5f), index, coords);
    grid.marker(index.x, index.y, index.z) = FLUID_CELL;
  }

  // mark solid cells
  int x, y, z;
  // top and bottom (y axis)
  for (int i = 0; i < grid.marker.sx; i++) {
    for (int k = 0; k < grid.marker.sy; k++) {
      grid.marker(i, grid.marker.sy - 1, k) = SOLID_CELL;
      grid.marker(i, 0, k) = SOLID_CELL;
    }
  }

  // left and right (x axis)
  for (int j = 0; j < grid.marker.sy; j++) {
    for (int k = 0; k < grid.marker.sz; k++) {
      grid.marker(grid.marker.sx - 1, j, k) = SOLID_CELL;
      grid.marker(0, j, k) = SOLID_CELL;
    }
  }

  // front and back (z axis)
  for (int i = 0; i < grid.marker.sx; i++) {
    for (int j = 0; j < grid.marker.sy; j++) {
      grid.marker(i, j, grid.marker.sz - 1) = SOLID_CELL;
      grid.marker(i, j, 0) = SOLID_CELL;
    }
  }
}

void Simulation::advance(float dt) {
  for (int i = 0; i < 5; i++)
    advect(0.2 * dt);
  particles_to_grid();
  save_velocities();
  grid.add_gravity(dt);
  mark_cells();
  grid.compute_phi();
  grid.extend_velocity();
  grid.enforce_boundary();
  grid.project(dt);
  grid.extend_velocity();
  grid_to_particles();
}

void Simulation::step_frame(float time) {
  float t = 0;
  float dt;
  while (t < time) {
    dt = std::fmin(grid.CFL(), 0.05f);
    if (t + dt >= time) {
      dt = time - t;
    }
    advance(dt);
    t += dt;
  }
}

void Simulation::save_particles(std::string fname) {
  std::ofstream ofile(std::string("out/p_") + fname);
  // header
  ofile << "ply\n";
  ofile << "format ascii 1.0\n";
  ofile << "element vertex " << particles.size() << "\n";
  ofile << "property float x\n";
  ofile << "property float y\n";
  ofile << "property float z\n";
  ofile << "end_header\n";

  for (auto &p : particles) {
    ofile << p.position.x << " " << p.position.z << " " << p.position.y << "\n";
  }
  ofile.close();
  std::cout << "   saved " << std::string("out/") + fname << std::endl;
}

void Simulation::save_voxels(std::string fname) {
  std::ofstream ofile(std::string("out/") + fname);
  int n = 0;
  for (int x = 0; x < grid.marker.size; x++) {
    if (grid.marker.data[x] == FLUID_CELL)
      n++;
  }
  // header
  ofile << "ply\n";
  ofile << "format ascii 1.0\n";
  ofile << "element vertex " << n << "\n";
  ofile << "property float x\n";
  ofile << "property float y\n";
  ofile << "property float z\n";
  ofile << "end_header\n";

  for (int i = 0; i < grid.marker.sx; i++) {
    for (int j = 0; j < grid.marker.sy; j++) {
      for (int k = 0; k < grid.marker.sz; k++) {
        if (grid.marker(i, j, k) == FLUID_CELL)
          ofile << i * grid.h << " " << k * grid.h << " " << j * grid.h << "\n";
      }
    }
  }
  ofile.close();
  std::cout << "   saved " << std::string("out/") + fname << std::endl;
}

void Simulation::step_and_save(float t, std::string fname) {
  float tm = 0.0f;
  float tstep = 0.05f;
  while (tm + tstep < t) {
    tm += tstep;
    std::cout << "   t: " << tm << std::endl;
    step_frame(tstep);
    save_particles(fname + std::string("_") + std::to_string(tm) +
                   std::string(".ply"));
  }
  if (tm < t) {
    std::cout << "   t: " << t << std::endl;
    step_frame(t - tm);
    save_particles(fname + std::string("_") + std::to_string(t) +
                   std::string(".ply"));
  }
}