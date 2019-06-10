#include "simulation.h"
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
  for (int x = 1; x < grid.nx - 1; x++) {
    for (int y = grid.ny * 0.75f; y < grid.ny - 1; y++) {
      for (int z = 1; z < grid.nz - 1; z++) {
        // for each cell, add 8 new jittered particles
        float base_x = x * grid.h;
        float base_y = y * grid.h;
        float base_z = z * grid.h;
        for (int i = 0; i < 8; i++) {
          float jitter_x = glm::linearRand(0 + EPS, grid.h - EPS);
          float jitter_y = glm::linearRand(0 + EPS, grid.h - EPS);
          float jitter_z = glm::linearRand(0 + EPS, grid.h - EPS);
          particles.emplace_back(Particle(glm::vec3(
              base_x + jitter_x, base_y + jitter_y, base_z + jitter_z)));
        }
      }
    }
  }
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
void Simulation::grid_add_quantities(Array3f &arr, float q, glm::ivec3 index,
                                     glm::vec3 coords) {
  float w;

  w = (1 - coords.x) * (1 - coords.y) * (1 - coords.z);
  arr(index.x, index.y, index.z) += w * q;
  grid.count(index.x, index.y, index.z) += w;

  w = (1 - coords.x) * (1 - coords.y) * (coords.z);
  arr(index.x, index.y, index.z + 1) += w * q;
  grid.count(index.x, index.y, index.z + 1) += w;

  w = (1 - coords.x) * (coords.y) * (1 - coords.z);
  arr(index.x, index.y + 1, index.z) += w * q;
  grid.count(index.x, index.y + 1, index.z) += w;

  w = (1 - coords.x) * (coords.y) * (coords.z);
  arr(index.x, index.y + 1, index.z + 1) += w * q;
  grid.count(index.x, index.y + 1, index.z + 1) += w;

  w = (coords.x) * (1 - coords.y) * (1 - coords.z);
  arr(index.x + 1, index.y, index.z) += w * q;
  grid.count(index.x + 1, index.y, index.z) += w;

  w = (coords.x) * (1 - coords.y) * (coords.z);
  arr(index.x + 1, index.y, index.z + 1) += w * q;
  grid.count(index.x + 1, index.y, index.z + 1) += w;

  w = (coords.x) * (coords.y) * (1 - coords.z);
  arr(index.x + 1, index.y + 1, index.z) += w * q;
  grid.count(index.x + 1, index.y + 1, index.z) += w;

  w = (coords.x) * (coords.y) * (coords.z);
  arr(index.x + 1, index.y + 1, index.z + 1) += w * q;
  grid.count(index.x + 1, index.y + 1, index.z + 1) += w;
}

// for each particle, trilinearly interpolate velocity
// to all grid points nearby
void Simulation::particles_to_grid() {
  // rho
  grid.rho.clear();
  for (auto p : particles) {
    glm::ivec3 index;
    glm::vec3 coords;
    position_to_grid(p.position, glm::vec3(0, 0, 0), index, coords);
    grid_add_quantities(grid.rho, 1.0f, index, coords);
  }

  // u
  grid.u.clear();
  grid.count.clear();
  for (auto p : particles) {
    glm::ivec3 index;
    glm::vec3 coords;
    position_to_grid(p.position, U_OFFSET, index, coords);
    grid_add_quantities(grid.u, p.velocity.x, index, coords);
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
  for (auto p : particles) {
    glm::ivec3 index;
    glm::vec3 coords;
    position_to_grid(p.position, V_OFFSET, index, coords);
    grid_add_quantities(grid.v, p.velocity.y, index, coords);
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
  for (auto p : particles) {
    glm::ivec3 index;
    glm::vec3 coords;
    position_to_grid(p.position, W_OFFSET, index, coords);
    grid_add_quantities(grid.w, p.velocity.z, index, coords);
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

void Simulation::grid_to_particles() {
  for (uint i = 0; i < particles.size(); i++) {
    Particle &p = particles[i];
    p.velocity = trilerp_uvw(p.position);
  }
}

void Simulation::advect(float dt) {
  glm::vec3 mid, gu;
  // boundaries
  float xmin = 1.001 * grid.h;
  float ymin = 1.001 * grid.h;
  float zmin = 1.001 * grid.h;
  float xmax = grid.lx - 1.001 * grid.h;
  float ymax = grid.ly - 1.001 * grid.h;
  float zmax = grid.lz - 1.001 * grid.h;
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
    p.position.x = glm::clamp(mid.x, xmin, xmax);
    p.position.y = glm::clamp(mid.y, ymin, ymax);
    p.position.z = glm::clamp(mid.z, zmin, zmax);
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
  y = grid.marker.sy - 1;
  for (int i = 0; i < grid.marker.sx; i++) {
    for (int k = 0; k < grid.marker.sy; k++) {
      grid.marker(i, y, k) = SOLID_CELL;
    }
  }
  y = 0;
  for (int i = 0; i < grid.marker.sx; i++) {
    for (int k = 0; k < grid.marker.sy; k++) {
      grid.marker(i, y, k) = SOLID_CELL;
    }
  }

  // left and right (x axis)
  x = grid.marker.sx - 1;
  for (int j = 0; j < grid.marker.sy; j++) {
    for (int k = 0; k < grid.marker.sz; k++) {
      grid.marker(x, j, k) = SOLID_CELL;
    }
  }
  x = 0;
  for (int j = 0; j < grid.marker.sy; j++) {
    for (int k = 0; k < grid.marker.sz; k++) {
      grid.marker(x, j, k) = SOLID_CELL;
    }
  }

  // front and back (z axis)
  z = grid.marker.sz - 1;
  for (int i = 0; i < grid.marker.sx; i++) {
    for (int j = 0; j < grid.marker.sy; j++) {
      grid.marker(i, j, z) = SOLID_CELL;
    }
  }
  z = 0;
  for (int i = 0; i < grid.marker.sx; i++) {
    for (int j = 0; j < grid.marker.sy; j++) {
      grid.marker(i, j, z) = SOLID_CELL;
    }
  }
}

void Simulation::advance(float dt) {
  particles_to_grid();  // done
  grid.add_gravity(dt); // done
  mark_cells();         // done
  grid.compute_phi();   // done
  // grid.extend_velocity();  // done
  grid.enforce_boundary(); // done
  // grid.project();          // todo
  // grid.extend_velocity(); // done
  grid_to_particles(); // done
  for (int i = 0; i < 5; i++)
    advect(0.2 * dt); // done
}

void Simulation::step_frame(float time) {
  float t = 0;
  float dt;
  while (t < time) {
    dt = grid.CFL();
    if (t + dt >= time) {
      dt = time - t;
    }
    advance(dt);
    t += dt;
  }
}