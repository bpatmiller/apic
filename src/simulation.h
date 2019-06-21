#pragma once

#include "Particle.h"
#include "grid.h"
#include "marchingcubes.h"
#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/common.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#include <vector>

#define PIC_MODE 0
#define PIC_FLIP_MODE 1
#define APIC_MODE 2

#define DAM_BREAK 0
#define CENTER_DROP 1
#define OPPOSITE_CORNERS 2

class Simulation {
public:
  // data
  Grid grid;
  std::vector<Particle> particles;
  std::vector<glm::vec3> cx;
  std::vector<glm::vec3> cy;
  std::vector<glm::vec3> cz;
  std::vector<glm::mat3x3> d; // using to validate the weight gradient
  // settings
  int mode = APIC_MODE;
  int example_type = DAM_BREAK;
  float flip_blend = 0.99f;
  // mesh data
  std::vector<glm::vec3> vertices;
  std::vector<glm::uvec3> indices;

  Simulation(){};

  void init(float lx_, int nx_, int ny_, int nz_) {
    grid.init(lx_, nx_, ny_, nz_);
  }

  void reset() {
    populate_particles();
    grid.reset();
  };

  // io methods
  void save_particles(std::string fname);
  void save_voxels(std::string fname);
  void generate_mesh();
  void save_mesh(std::string fname);
  // particle init methods
  void populate_particles();
  void add_dam_break();
  void add_center_drop();
  void add_opp_corners();
  // auxillary methods
  void step_and_save(float t, std::string fname);
  void advance(float dt);
  void step_frame(float time);
  // simulation methods
  void particles_to_grid();
  void save_velocities();
  void grid_to_particles();
  void advect(float dt);
  void mark_cells();
  // helper functions
  void position_to_grid(glm::vec3 p, glm::vec3 offset, glm::ivec3 &index,
                        glm::vec3 &coords);
  template <class T>
  void grid_add_quantities(T &arr, float q, glm::ivec3 index, glm::vec3 coords);
  glm::vec3 trilerp_uvw(glm::vec3 p);
  glm::vec3 trilerp_dudvdw(glm::vec3 p);
  // APIC functions
  template <class T>
  void affine_set(T &accum, glm::vec3 c, glm::ivec3 index, glm::vec3 coords);
  glm::vec3 compute_C(Array3f &field, glm::ivec3 index, glm::vec3 coords);
};