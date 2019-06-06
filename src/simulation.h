#pragma once

#include "Particle.h"
#include "grid.h"
#include <glm/glm.hpp>
#include <vector>

class Simulation {
public:
  Grid grid;
  std::vector<Particle> particles;

  Simulation(){};

  void init(float lx_, int nx_, int ny_, int nz_) {
    grid.init(lx_, nx_, ny_, nz_);
  }

  float CFL();
  void add_particle_box();
  void particles_to_grid();
  void grid_to_particles();
  void advect(float dt);
  void advance(float dt);
  void step_frame(float time);

  // helper functions
  glm::ivec3 position_to_grid_index(glm::vec3 p, glm::vec3 offset);
  glm::ivec3 position_to_lower_grid_index(glm::vec3 p, glm::vec3 offset);
  glm::vec3 position_to_lower_grid_coords(glm::vec3 p, glm::vec3 offset);
  void grid_add_quantities(Array3f &arr, float q, glm::ivec3 index,
                           glm::vec3 coords);
};