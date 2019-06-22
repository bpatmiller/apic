#include "simulation.h"

#define U_OFFSET glm::vec3(0.5f, 0, 0)
#define V_OFFSET glm::vec3(0, 0.5f, 0)
#define W_OFFSET glm::vec3(0, 0, 0.5f)

#define EPS 0.001

void Simulation::populate_particles() {
  reseed_count = 0;
  particles.clear();
  cx.clear();
  cy.clear();
  cz.clear();
  // add_dam_break();
  add_center_drop();
}

void Simulation::reseed_cell(int i, int j, int k) {
  if (grid.marker(i, j, k) != SOLID_CELL) {
    float base_x = i * grid.h;
    float base_y = j * grid.h;
    float base_z = k * grid.h;
    for (int i = 0; i < 8; i++) {
      float jitter_x = glm::linearRand(0 + EPS, grid.h - EPS);
      float jitter_y = glm::linearRand(0 + EPS, grid.h - EPS);
      float jitter_z = glm::linearRand(0 + EPS, grid.h - EPS);
      // add particles
      particles.push_back(Particle(
          glm::vec3(base_x + jitter_x, base_y + jitter_y, base_z + jitter_z),
          glm::vec3(0, 0, 0)));
      // APIC vectors
      cx.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
      cy.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
      cz.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
    }
    dirty = true;
  }
}

void Simulation::reseed_particles() {
  reseed_count = 0;
  particles.clear();
  cx.clear();
  cy.clear();
  cz.clear();
  for (int i = 0; i < grid.marker.sx; i++) {
    for (int j = 0; j < grid.marker.sy; j++) {
      for (int k = 0; k < grid.marker.sz; k++) {
        if (grid.marker(i, j, k) == FLUID_CELL) {
          reseed_cell(i, j, k);
        }
      }
    }
  }
}

void Simulation::emit_particles() {
  for (auto &e : emitters) {
    for (int i = 0, imax = (int)e.rate; i < imax; i++) {
      float base_x = e.position.x;
      float base_y = e.position.y;
      float base_z = e.position.z;
      for (int i = 0; i < 8; i++) {
        float jitter_x = glm::linearRand(-e.radius, e.radius) * e.scale.x;
        float jitter_y = glm::linearRand(-e.radius, e.radius) * e.scale.y;
        float jitter_z = glm::linearRand(-e.radius, e.radius) * e.scale.z;
        // add particles
        particles.push_back(Particle(
            glm::vec3(base_x + jitter_x, base_y + jitter_y, base_z + jitter_z),
            e.direction));
        // APIC vectors
        cx.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
        cy.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
        cz.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
      }
    }
  }
}

void Simulation::add_dam_break() {
  for (int x = grid.nx * 0.75; x < grid.nx * 0.95; x++) {
    for (int y = grid.ny * 0.1; y < grid.ny * 0.6; y++) {
      for (int z = grid.nz * 0.1; z < grid.nz * 0.9; z++) {
        // for each cell, add 8 new jittered particles
        reseed_cell(x, y, z);
      }
    }
  }
}

void Simulation::add_center_drop() {
  for (int x = grid.nx * 0.3; x < grid.nx * 0.7; x++) {
    for (int y = grid.ny * 0.5; y < grid.ny * 0.8; y++) {
      for (int z = grid.nz * 0.3; z < grid.nz * 0.7; z++) {
        // for each cell, add 8 new jittered particles
        reseed_cell(x, y, z);
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
  for (int i = 0; i <= 1; i++) { // {0,1}
    for (int j = 0; j <= 1; j++) {
      for (int k = 0; k <= 1; k++) {
        float w = (1 - std::fabs(i - coords.x)) * // {1 - coords.x | coords.x}
                  (1 - std::fabs(j - coords.y)) * // {""}
                  (1 - std::fabs(k - coords.z));  //{""}
        arr(index.x + i, index.y + j, index.z + k) += w * q; //{0,1}
        grid.count(index.x + i, index.y + j, index.z + k) += w;
      }
    }
  }
}

template <class T>
void Simulation::grid_add_quantities_constant(T &arr, float q, glm::ivec3 index,
                                              glm::vec3 coords) {
  for (int i = 0; i <= 1; i++) {
    for (int j = 0; j <= 1; j++) {
      for (int k = 0; k <= 1; k++) {
        arr(index.x + i, index.y + j, index.z + k) += q;
      }
    }
  }
}

template <class T>
void Simulation::affine_set(T &accum, glm::vec3 c, glm::ivec3 index,
                            glm::vec3 coords) {
  for (int i = 0; i <= 1; i++) { // {0,1}
    for (int j = 0; j <= 1; j++) {
      for (int k = 0; k <= 1; k++) {
        float w = (1 - std::fabs(i - coords.x)) * // {1 - coords.x | coords.x}
                  (1 - std::fabs(j - coords.y)) * // {""}
                  (1 - std::fabs(k - coords.z));  //{""}
        float coef = glm::dot(
            c, glm::vec3(i - coords.x, j - coords.y, k - coords.z) * grid.h);
        accum(index.x + i, index.y + j, index.z + k) += w * coef; //{0,1}
      }
    }
  }
}

// for each particle, trilinearly interpolate velocity
// to all grid points nearby
void Simulation::particles_to_grid() {
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
  glm::vec3 wg;
  glm::vec3 c = glm::vec3(0.0f, 0.0f, 0.0f);
  glm::mat3x3 D = glm::mat3x3(0.0f);

  // offset position slightly to prevent a singular D
  coords = glm::clamp(coords, 0.0000001f, 0.9999999f);

  // compute D
  for (int i = 0; i <= 1; i++) { // {0,1}
    for (int j = 0; j <= 1; j++) {
      for (int k = 0; k <= 1; k++) {
        float w = (1 - std::fabs(i - coords.x)) * // {1 - coords.x | coords.x}
                  (1 - std::fabs(j - coords.y)) * // {""}
                  (1 - std::fabs(k - coords.z));  //{""}
        glm::vec3 v =
            glm::vec3(i - coords.x, j - coords.y, k - coords.z) * grid.h;

        D += w * glm::outerProduct(v, v);
      }
    }
  }

  // weight gradient = w * D^-1 * (x_i - x_p)
  for (int i = 0; i <= 1; i++) { // {0,1}
    for (int j = 0; j <= 1; j++) {
      for (int k = 0; k <= 1; k++) {
        float w = (1 - std::fabs(i - coords.x)) * // {1 - coords.x | coords.x}
                  (1 - std::fabs(j - coords.y)) * // {""}
                  (1 - std::fabs(k - coords.z));  //{""}
        glm::vec3 v =
            glm::vec3(i - coords.x, j - coords.y, k - coords.z) * grid.h;
        assert(glm::determinant(D) != 0);
        wg = w * glm::inverse(D) * glm::vec3(v);
        c += wg * field(index.x + i, index.y + j, index.z + k);
      }
    }
  }

  return c;
}

void Simulation::grid_to_particles() {
  // pic flip velocity differences
  if (mode == PIC_FLIP_MODE) {
    for (int nu = 0; nu < grid.u.size; nu++) {
      grid.du.data[nu] = grid.u.data[nu] - grid.du.data[nu];
    }
    for (int nv = 0; nv < grid.v.size; nv++) {
      grid.dv.data[nv] = grid.v.data[nv] - grid.dv.data[nv];
    }
    for (int nw = 0; nw < grid.w.size; nw++) {
      grid.dw.data[nw] = grid.w.data[nw] - grid.dw.data[nw];
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

    // push out of solid obstacles
    glm::vec3 coords;
    glm::ivec3 index;
    position_to_grid(p.position, glm::vec3(0.5, 0.5, 0.5), index, coords);
    if (grid.marker(index.x, index.y, index.z) == SOLID_CELL) {
      // reesed particles elsewhere
      glm::ivec4 best_candidate = candidates.back();
      p.position = glm::vec3((best_candidate.x + 0.5f) * grid.h,
                             (best_candidate.y + 0.5f) * grid.h,
                             (best_candidate.z + 0.5f) * grid.h);
      // resample velocity and C
      p.velocity = trilerp_uvw(p.position);
      cx[i] = glm::vec3(0);
      cy[i] = glm::vec3(0);
      cz[i] = glm::vec3(0);
      candidates.pop_back();
    }
  }
}

void Simulation::mark_cells() {
  // mark all non-solid cells as empty
  for (int i = 0; i < grid.marker.size; i++) {
    if (grid.marker.data[i] != SOLID_CELL) {
      grid.marker.data[i] = AIR_CELL;
    }
  }

  // mark liquid cells
  glm::ivec3 index;
  glm::vec3 coords;
  for (Particle &p : particles) {
    position_to_grid(p.position, glm::vec3(0.5f, 0.5f, 0.5f), index, coords);
    grid.marker(index.x, index.y, index.z) = FLUID_CELL;
  }
}

void Simulation::intialize_boundaries() {
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

void Simulation::make_candidate_reseeds() {
  // populate pc array with effect of each particle
  // on each grid node
  for (auto &p : particles) {
    glm::vec3 coords;
    glm::ivec3 index;
    position_to_grid(p.position, glm::vec3(0.5, 0.5, 0.5), index, coords);
    grid_add_quantities_constant(grid.pc, 1, index, coords);
  }

  // loop through all fluid cells, if they
  // are surrounded on all sides by fluid,
  // add them to a candidate list sor
  candidates.clear();
  for (int i = 0; i < grid.marker.sx; i++) {
    for (int j = 0; j < grid.marker.sy; j++) {
      for (int k = 0; k < grid.marker.sz; k++) {
        if (grid.marker(i, j, k) == FLUID_CELL &&
            grid.marker(i + 1, j, k) == FLUID_CELL &&
            grid.marker(i - 1, j, k) == FLUID_CELL &&
            grid.marker(i, j + 1, k) == FLUID_CELL &&
            grid.marker(i, j - 1, k) == FLUID_CELL &&
            grid.marker(i, j, k + 1) == FLUID_CELL &&
            grid.marker(i, j, k - 1) == FLUID_CELL) {
          candidates.push_back(glm::ivec4(i, j, k, grid.pc(i, j, k)));
        }
      }
    }
  }
  // sort with highest pc first
  // and later pop candidates during reseed
  std::sort(
      candidates.begin(), candidates.end(),
      [](const glm::ivec4 &a, const glm::ivec4 &b) { return a[3] > b[3]; });
}

void Simulation::advance(float dt) {
  reseed_count = 0;
  emit_particles();
  make_candidate_reseeds();
  for (int i = 0; i < 5; i++)
    advect(0.2 * dt);
  particles_to_grid();
  grid.save_velocity();
  grid.add_gravity(dt);
  mark_cells();
  grid.compute_phi();
  grid.extend_velocity();
  grid.enforce_boundary();
  grid.project(dt);
  grid.extend_velocity();
  grid_to_particles();
  // std::printf("(%f%%) reseed count: %i\n",
  //             100.0f * (float)reseed_count / (float)particles.size(),
  //             reseed_count);
}

void Simulation::step_frame(float time) {
  float t = 0;
  float dt;
  while (t < time) {
    dt = 2.0 * grid.CFL();
    if (t + dt >= time) {
      dt = time - t;
    }
    advance(dt);
    t += dt;
  }
}

void Simulation::step_and_save(float t, std::string fname) {
  float tm = 0.0f;
  float tstep = 0.05f;
  while (tm < t) {
    if (tm + tstep > t)
      tstep = t - tm;
    tm += tstep;
    std::cout << "   t: " << tm << std::endl;
    step_frame(tstep);
    generate_mesh();
    save_mesh(fname + std::string("_") + std::to_string(tm) +
              std::string(".ply"));
  }
}
