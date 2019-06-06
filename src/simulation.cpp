#include "simulation.h"
#include <glm/gtc/random.hpp>
#include <glm/gtx/common.hpp>

#define U_OFFSET glm::vec3(-grid.h * 0.5, 0, 0)
#define V_OFFSET glm::vec3(0, -grid.h * 0.5, 0)
#define W_OFFSET glm::vec3(0, 0, -grid.h * 0.5)
#define EPS 0.001

// initialize "dam break" scenario
void Simulation::add_particle_box() {
  for (int x = grid.nx / 2; x < grid.nx - 1; x++) {
    for (int y = 1; y < grid.ny - 1; y++) {
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

// returns the closest grid index (with an offset)
// defaults to sampling center of grids (position
// given by vec3(h/2) + (h*x, h*y, h*z)
// e.g. snapping to the nearest u point given by:
// position_to_grid_index(p, vec3(-h/2, 0, 0))
// in which position is given by
// (0, h/2, h/2) + (h*x, h*y, h*z)
glm::ivec3 Simulation::position_to_grid_index(glm::vec3 p, glm::vec3 offset) {
  int i = glm::round((p.x - grid.h * 0.5 - offset.x) / grid.h);
  int j = glm::round((p.y - grid.h * 0.5 - offset.y) / grid.h);
  int k = glm::round((p.z - grid.h * 0.5 - offset.z) / grid.h);
  return glm::ivec3(i, j, k);
}

// does the same as above, but rounds down for easier grid transfer
glm::ivec3 Simulation::position_to_lower_grid_index(glm::vec3 p,
                                                    glm::vec3 offset) {
  int i = static_cast<int>((p.x - grid.h * 0.5 - offset.x) / grid.h);
  int j = static_cast<int>((p.y - grid.h * 0.5 - offset.y) / grid.h);
  int k = static_cast<int>((p.z - grid.h * 0.5 - offset.z) / grid.h);
  return glm::ivec3(i, j, k);
}

glm::vec3 Simulation::position_to_lower_grid_coords(glm::vec3 p,
                                                    glm::vec3 offset) {
  float x =
      glm::fmod(static_cast<float>(p.x - grid.h * 0.5 - offset.x), grid.h);
  float y =
      glm::fmod(static_cast<float>(p.x - grid.h * 0.5 - offset.x), grid.h);
  float z =
      glm::fmod(static_cast<float>(p.x - grid.h * 0.5 - offset.x), grid.h);
  return glm::vec3(x, y, z);
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
  // u
  grid.u.clear();
  grid.count.clear();
  for (auto p : particles) {
    glm::ivec3 index = position_to_lower_grid_index(p.position, U_OFFSET);
    glm::vec3 coords = position_to_lower_grid_coords(p.position, U_OFFSET);
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
    glm::ivec3 index = position_to_lower_grid_index(p.position, V_OFFSET);
    glm::vec3 coords = position_to_lower_grid_coords(p.position, V_OFFSET);
    grid_add_quantities(grid.v, p.velocity.y, index, coords);
  }
  // average velocities
  for (int i = 0; i < grid.u.sx; i++) {
    for (int j = 0; j < grid.u.sy; j++) {
      for (int k = 0; k < grid.u.sz; k++) {
        if (grid.count(i, j, k) != 0)
          grid.v(i, j, k) /= grid.count(i, j, k);
      }
    }
  }

  // w
  grid.w.clear();
  grid.count.clear();
  for (auto p : particles) {
    glm::ivec3 index = position_to_lower_grid_index(p.position, W_OFFSET);
    glm::vec3 coords = position_to_lower_grid_coords(p.position, W_OFFSET);
    grid_add_quantities(grid.w, p.velocity.z, index, coords);
  }
  // average velocities
  for (int i = 0; i < grid.u.sx; i++) {
    for (int j = 0; j < grid.u.sy; j++) {
      for (int k = 0; k < grid.u.sz; k++) {
        if (grid.count(i, j, k) != 0)
          grid.w(i, j, k) /= grid.count(i, j, k);
      }
    }
  }
}

void Simulation::grid_to_particles() {
  for (uint i = 0; i < particles.size(); i++) {
    Particle &p = particles[i];
  }
}

void Simulation::advect(float dt) {}

void Simulation::advance(float dt) {
  for (int i = 0; i < 5; i++)
    advect(0.2 * dt);
  particles_to_grid();
  grid.add_gravity(dt);
}

void Simulation::step_frame() {}