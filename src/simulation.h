#pragma once

#include "Particle.h"
#include "grid.h"
#include <glm/glm.hpp>
#include <vector>

#define PIC_MODE 0
#define PIC_FLIP_MODE 1
#define APIC_MODE 2

class Simulation {
public:
  Grid grid;
  std::vector<Particle> particles;
  int mode = 1;
  float flip_blend = 0.25f;

  Simulation(){};

  void init(float lx_, int nx_, int ny_, int nz_) {
    grid.init(lx_, nx_, ny_, nz_);
  }

  void reset() {
    add_particle_box();
    grid.reset();
  };

  void add_particle_box();
  void particles_to_grid();
  void save_velocities();
  void grid_to_particles();
  void advect(float dt);
  void advance(float dt);
  void step_frame(float time);
  void mark_cells();

  // helper functions
  void position_to_grid(glm::vec3 p, glm::vec3 offset, glm::ivec3 &index,
                        glm::vec3 &coords);
  void grid_add_quantities(Array3f &arr, float q, glm::ivec3 index,
                           glm::vec3 coords);
  glm::vec3 trilerp_uvw(glm::vec3 p);
  glm::vec3 trilerp_dudvdw(glm::vec3 p);
};