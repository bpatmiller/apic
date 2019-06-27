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

#define U_OFFSET glm::vec3(0.5f, 0, 0)
#define V_OFFSET glm::vec3(0, 0.5f, 0)
#define W_OFFSET glm::vec3(0, 0, 0.5f)
#define CENTER_OFFSET glm::vec3(0.5f, 0.5f, 0.5f)

#define EPS 0.001

enum SimType { PIC = 1, PIC_FLIP = 2, APIC = 3 };

class Emitter {
public:
  Emitter(){};
  Emitter(glm::vec3 p, glm::vec3 d, glm::vec3 s, float r, float rad) {
    position = p;
    direction = d;
    scale = s;
    rate = r;
    radius = rad;
  }
  glm::vec3 position;
  glm::vec3 direction;
  glm::vec3 scale; // physical scale of object
  float rate;
  float radius;
};

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
  SimType mode = APIC;
  float flip_blend = 0.99f;
  bool dirty = true;
  // reseed data
  std::vector<glm::ivec4> candidates;
  int reseed_count = 0;
  bool reseed = false;
  // mesh data
  std::vector<glm::vec3> vertices;
  std::vector<glm::uvec3> indices;
  // emitter data
  std::vector<Emitter> emitters;

  Simulation(){};

  void init(float lx_, int nx_, int ny_, int nz_) {
    grid.init(lx_, nx_, ny_, nz_);
    intialize_boundaries();
  }

  void reset() {
    grid.reset();
    intialize_boundaries();
    populate_particles();
  };

  // io methods
  void save_particles(std::string fname);
  void save_voxels(std::string fname);
  void generate_mesh();
  void save_mesh(std::string fname);
  // particle init methods
  void populate_particles();
  void reseed_particles();
  void reseed_cell(int i, int j, int k, int id = 1, float mass = 1.0f,
                   float viscosity = 1.0f);
  void add_dam_break();
  void add_center_drop();
  void add_pool();
  void add_raleigh_taylor();
  void emit_particles();
  // auxillary methods
  void intialize_boundaries();
  void step_and_save(float t, std::string fname);
  void advance(float dt);
  void step_frame(float time);
  void make_candidate_reseeds();
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
  void grid_add_quantities(T &arr, T &weights, float q, glm::ivec3 index,
                           glm::vec3 coords, float mass);
  template <class T>
  void grid_add_quantities_constant(T &arr, float q, glm::ivec3 index,
                                    glm::vec3 coords);
  glm::vec3 trilerp_uvw(glm::vec3 p);
  glm::vec3 trilerp_dudvdw(glm::vec3 p);
  // APIC functions
  template <class T>
  void affine_set(T &accum, glm::vec3 c, glm::ivec3 index, glm::vec3 coords,
                  float mass);
  glm::vec3 compute_C(Array3f &field, glm::ivec3 index, glm::vec3 coords,
                      float mass);
};